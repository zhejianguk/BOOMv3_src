//******************************************************************************
// Copyright (c) 2017 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

package boom.common

import chisel3._
import chisel3.util.{RRArbiter, Queue}

import scala.collection.mutable.{ListBuffer}

import freechips.rocketchip.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.diplomaticobjectmodel.logicaltree.{LogicalTreeNode }
import freechips.rocketchip.rocket._
import freechips.rocketchip.subsystem.{RocketCrossingParams}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._

import testchipip.{ExtendedTracedInstruction, WithExtendedTraceport}

import boom.exu._
import boom.ifu._
import boom.lsu._
import boom.util.{BoomCoreStringPrefix}
import freechips.rocketchip.prci.ClockSinkParameters
//===== GuardianCouncil Function: Start ====//
import freechips.rocketchip.guardiancouncil._
//===== GuardianCouncil Function: End   ====//

case class BoomTileAttachParams(
  tileParams: BoomTileParams,
  crossingParams: RocketCrossingParams
) extends CanAttachTile {
  type TileType = BoomTile
  val lookup = PriorityMuxHartIdFromSeq(Seq(tileParams))
}


/**
 * BOOM tile parameter class used in configurations
 *
 */
case class BoomTileParams(
  core: BoomCoreParams = BoomCoreParams(),
  icache: Option[ICacheParams] = Some(ICacheParams()),
  dcache: Option[DCacheParams] = Some(DCacheParams()),
  btb: Option[BTBParams] = Some(BTBParams()),
  trace: Boolean = false,
  name: Option[String] = Some("boom_tile"),
  hartId: Int = 0
) extends InstantiableTileParams[BoomTile]
{
  require(icache.isDefined)
  require(dcache.isDefined)
  def instantiate(crossing: TileCrossingParamsLike, lookup: LookupByHartIdImpl)(implicit p: Parameters): BoomTile = {
    new BoomTile(this, crossing, lookup)
  }
  val beuAddr: Option[BigInt] = None
  val blockerCtrlAddr: Option[BigInt] = None
  val boundaryBuffers: Boolean = false // if synthesized with hierarchical PnR, cut feed-throughs?
  val clockSinkParams: ClockSinkParameters = ClockSinkParameters()
}

/**
 * BOOM tile
 *
 */
class BoomTile private(
  val boomParams: BoomTileParams,
  crossing: ClockCrossingType,
  lookup: LookupByHartIdImpl,
  q: Parameters)
  extends BaseTile(boomParams, crossing, lookup, q)
  with SinksExternalInterrupts
  with SourcesExternalNotifications
  with WithExtendedTraceport
{

  // Private constructor ensures altered LazyModule.p is used implicitly
  def this(params: BoomTileParams, crossing: TileCrossingParamsLike, lookup: LookupByHartIdImpl)(implicit p: Parameters) =
    this(params, crossing.crossingType, lookup, p)

  val intOutwardNode = IntIdentityNode()
  val masterNode = visibilityNode
  val slaveNode = TLIdentityNode()

  val tile_master_blocker =
    tileParams.blockerCtrlAddr
      .map(BasicBusBlockerParams(_, xBytes, masterPortBeatBytes, deadlock = true))
      .map(bp => LazyModule(new BasicBusBlocker(bp)))

  tile_master_blocker.foreach(lm => connectTLSlave(lm.controlNode, xBytes))

  // TODO: this doesn't block other masters, e.g. RoCCs
  tlOtherMastersNode := tile_master_blocker.map { _.node := tlMasterXbar.node } getOrElse { tlMasterXbar.node }
  masterNode :=* tlOtherMastersNode

  val cpuDevice: SimpleDevice = new SimpleDevice("cpu", Seq("ucb-bar,boom0", "riscv")) {
    override def parent = Some(ResourceAnchors.cpus)
    override def describe(resources: ResourceBindings): Description = {
      val Description(name, mapping) = super.describe(resources)
      Description(name, mapping ++
                        cpuProperties ++
                        nextLevelCacheProperty ++
                        tileProperties)
    }
  }

  ResourceBinding {
    Resource(cpuDevice, "reg").bind(ResourceAddress(staticIdForMetadataUseOnly))
  }

  override def makeMasterBoundaryBuffers(crossing: ClockCrossingType)(implicit p: Parameters) = crossing match {
    case _: RationalCrossing =>
      if (!boomParams.boundaryBuffers) TLBuffer(BufferParams.none)
      else TLBuffer(BufferParams.none, BufferParams.flow, BufferParams.none, BufferParams.flow, BufferParams(1))
    case _ => TLBuffer(BufferParams.none)
  }

  override def makeSlaveBoundaryBuffers(crossing: ClockCrossingType)(implicit p: Parameters) = crossing match {
    case _: RationalCrossing =>
      if (!boomParams.boundaryBuffers) TLBuffer(BufferParams.none)
      else TLBuffer(BufferParams.flow, BufferParams.none, BufferParams.none, BufferParams.none, BufferParams.none)
    case _ => TLBuffer(BufferParams.none)
  }

  override lazy val module = new BoomTileModuleImp(this)

  // DCache
  lazy val dcache: BoomNonBlockingDCache = LazyModule(new BoomNonBlockingDCache(staticIdForMetadataUseOnly))
  val dCacheTap = TLIdentityNode()
  tlMasterXbar.node := dCacheTap := dcache.node


  // Frontend/ICache
  val frontend = LazyModule(new BoomFrontend(tileParams.icache.get, staticIdForMetadataUseOnly))
  frontend.resetVectorSinkNode := resetVectorNexusNode
  tlMasterXbar.node := frontend.masterNode

  // ROCC
  val roccs = p(BuildRoCC).map(_(p))
  roccs.map(_.atlNode).foreach { atl => tlMasterXbar.node :=* atl }
  roccs.map(_.tlNode).foreach { tl => tlOtherMastersNode :=* tl }
}

/**
 * BOOM tile implementation
 *
 * @param outer top level BOOM tile
 */
class BoomTileModuleImp(outer: BoomTile) extends BaseTileModuleImp(outer){

  Annotated.params(this, outer.boomParams)

  val core = Module(new BoomCore(outer.boomParams.trace)(outer.p))
  val lsu  = Module(new LSU()(outer.p, outer.dcache.module.edge))

  val ght_bridge = Module(new GH_Bridge(GH_BridgeParams(1)))
  val ghe_bridge = Module(new GH_Bridge(GH_BridgeParams(3)))
  val ght_cfg_bridge = Module(new GH_Bridge(GH_BridgeParams(32)))
  val ght_cfg_v_bridge = Module(new GH_Bridge(GH_BridgeParams(1)))

  //===== GuardianCouncil Function: Start ====//
  if (outer.tileParams.hartId == 0) {
    println("#### Jessica #### Generating GHT for the big core, HartID: ", outer.boomParams.hartId, "...!!!")
    val ght = Module(new GHT(GHTParams(vaddrBitsExtended, p(XLen), 32, 32, 16, 128)))   // revisit: set 32 as the total number of checkers.
                                                                                        // revisit: total types of insts is 32
                                                                                        // revisit: total number of SEs is 16 
                                                                                        // revisit: packet size: 128 bits
    ght.io.ght_pcaddr_in                         := core.io.pc
    ght.io.resetvector_in                        := outer.resetVectorSinkNode.bundle
    ght.io.ght_inst_in                           := core.io.inst
    ght.io.ght_alu_in                            := core.io.alu_2cycle_delay
    ght.io.ght_mask_in                           := ght_bridge.io.out
    ght.io.ght_cfg_in                            := ght_cfg_bridge.io.out
    ght.io.ght_cfg_valid                         := ght_cfg_v_bridge.io.out
    outer.ght_packet_out_SRNode.bundle           := ght.io.ght_packet_out
    outer.ght_packet_dest_SRNode.bundle          := ght.io.ght_packet_dest
    core.io.clk_enable_gh                        := ~(outer.bigcore_hang_in_SKNode.bundle) 
    outer.ghe_event_out_SRNode.bundle            := ghe_bridge.io.out
    ght.io.core_na                               := outer.sch_na_inSKNode.bundle
    ght.io.new_commit                            := core.io.new_commit
    ght.io.csr_rw_wdata                          := core.io.csr_rw_wdata
    outer.ghm_agg_core_id_out_SRNode.bundle      := ght.io.ghm_agg_core_id
  } else { 
    // Not be used, added to pass the compile
    core.io.clk_enable_gh                        := 1.U
  }

  val ptwPorts         = ListBuffer(lsu.io.ptw, outer.frontend.module.io.ptw, core.io.ptw_tlb)

  val hellaCachePorts  = ListBuffer[HellaCacheIO]()

  outer.reportWFI(None) // TODO: actually report this?

  outer.decodeCoreInterrupts(core.io.interrupts) // Decode the interrupt vector

  // Pass through various external constants and reports
  outer.extTraceSourceNode.bundle <> core.io.trace
  outer.traceSourceNode.bundle <> DontCare
  outer.bpwatchSourceNode.bundle <> DontCare // core.io.bpwatch
  core.io.hartid := outer.hartIdSinkNode.bundle

  // Connect the core pipeline to other intra-tile modules
  outer.frontend.module.io.cpu <> core.io.ifu
  core.io.lsu <> lsu.io.core

  //fpuOpt foreach { fpu => core.io.fpu <> fpu.io } RocketFpu - not needed in boom
  core.io.rocc := DontCare

  if (outer.roccs.size > 0) {
    val (respArb, cmdRouter) = {
      val respArb = Module(new RRArbiter(new RoCCResponse()(outer.p), outer.roccs.size))
      val cmdRouter = Module(new RoccCommandRouter(outer.roccs.map(_.opcodes))(outer.p))
      outer.roccs.zipWithIndex.foreach { case (rocc, i) =>
        ptwPorts ++= rocc.module.io.ptw
        rocc.module.io.cmd <> cmdRouter.io.out(i)
        val dcIF = Module(new SimpleHellaCacheIF()(outer.p))
        dcIF.io.requestor <> rocc.module.io.mem
        hellaCachePorts += dcIF.io.cache
        respArb.io.in(i) <> Queue(rocc.module.io.resp)
        //===== GuardianCouncil Function: Start ====//
        rocc.module.io.ghe_packet_in                 := cmdRouter.io.ghe_packet_in
        rocc.module.io.ghe_status_in                 := cmdRouter.io.ghe_status_in
        rocc.module.io.bigcore_comp                  := cmdRouter.io.bigcore_comp
        cmdRouter.io.ght_mask_in                     := rocc.module.io.ght_mask_out
        cmdRouter.io.ght_status_in                   := rocc.module.io.ght_status_out
        cmdRouter.io.ghe_event_in                    := rocc.module.io.ghe_event_out
        cmdRouter.io.ght_cfg_in                      := rocc.module.io.ght_cfg_out
        cmdRouter.io.ght_cfg_valid_in                := rocc.module.io.ght_cfg_valid

        cmdRouter.io.agg_packet_in                   := rocc.module.io.agg_packet_out
        rocc.module.io.agg_buffer_full               := cmdRouter.io.agg_buffer_full
        cmdRouter.io.agg_core_full_in                := rocc.module.io.agg_core_full
        cmdRouter.io.ght_sch_na_in                   := rocc.module.io.ght_sch_na
        rocc.module.io.ght_sch_refresh               := cmdRouter.io.ght_sch_refresh
        cmdRouter.io.ght_sch_dorefresh_in            := rocc.module.io.ght_sch_dorefresh
        //===== GuardianCouncil Function: End   ====//
      }
      // Create this FPU just for RoCC
      val nFPUPorts = outer.roccs.filter(_.usesFPU).size
      if (nFPUPorts > 0) {
        val fpuOpt = outer.tileParams.core.fpu.map(params => Module(new freechips.rocketchip.tile.FPU(params)(outer.p)))
        // TODO: Check this FPU works properly
        fpuOpt foreach { fpu =>
          // This FPU does not get CPU requests
          fpu.io := DontCare
          fpu.io.fcsr_rm := core.io.fcsr_rm
          fpu.io.dmem_resp_val := false.B
          fpu.io.valid := false.B
          fpu.io.killx := false.B
          fpu.io.killm := false.B

          val fpArb = Module(new InOrderArbiter(new FPInput()(outer.p), new FPResult()(outer.p), nFPUPorts))
          val fp_rocc_ios = outer.roccs.filter(_.usesFPU).map(_.module.io)
          fpArb.io.in_req <> fp_rocc_ios.map(_.fpu_req)
          fp_rocc_ios.zip(fpArb.io.in_resp).foreach {
            case (rocc, arb) => rocc.fpu_resp <> arb
          }
          fpu.io.cp_req <> fpArb.io.out_req
          fpArb.io.out_resp <> fpu.io.cp_resp
        }
      }
      (respArb, cmdRouter)
    }

    cmdRouter.io.in <> core.io.rocc.cmd
    outer.roccs.foreach(_.module.io.exception := core.io.rocc.exception)
    core.io.rocc.resp <> respArb.io.out
    core.io.rocc.busy <> (cmdRouter.io.busy || outer.roccs.map(_.module.io.busy).reduce(_||_))
    core.io.rocc.interrupt := outer.roccs.map(_.module.io.interrupt).reduce(_||_)
    //===== GuardianCouncil Function: Start ====//
    cmdRouter.io.ghe_packet_in                   := outer.ghe_packet_in_SKNode.bundle
    cmdRouter.io.ghe_status_in                   := outer.ghe_status_in_SKNode.bundle
    ghe_bridge.io.in                             := cmdRouter.io.ghe_event_out
    ght_bridge.io.in                             := cmdRouter.io.ght_mask_out
    ght_cfg_bridge.io.in                         := cmdRouter.io.ght_cfg_out
    ght_cfg_v_bridge.io.in                       := cmdRouter.io.ght_cfg_valid
    outer.ght_status_out_SRNode.bundle           := cmdRouter.io.ght_status_out

    // agg
    outer.agg_packet_out_SRNode.bundle           := cmdRouter.io.agg_packet_out
    cmdRouter.io.agg_buffer_full                 := outer.agg_buffer_full_in_SKNode.bundle
    outer.agg_core_full_SRNode.bundle            := cmdRouter.io.agg_core_full_out
    outer.ght_sch_na_out_SRNode.bundle           := cmdRouter.io.ght_sch_na_out
    cmdRouter.io.ght_sch_refresh                 := outer.ghe_sch_refresh_in_SKNode.bundle
    // For big_core GHT
    cmdRouter.io.bigcore_comp                    := outer.bigcore_comp_in_SKNode.bundle
    outer.ght_sch_dorefresh_SRNode.bundle        := cmdRouter.io.ght_sch_dorefresh_out                           
    //===== GuardianCouncil Function: End   ====//
  }

  // PTW
  val ptw  = Module(new PTW(ptwPorts.length)(outer.dcache.node.edges.out(0), outer.p))
  core.io.ptw <> ptw.io.dpath
  ptw.io.requestor <> ptwPorts
  hellaCachePorts += ptw.io.mem

   // LSU IO
  val hellaCacheArb = Module(new HellaCacheArbiter(hellaCachePorts.length)(outer.p))
  hellaCacheArb.io.requestor <> hellaCachePorts
  lsu.io.hellacache <> hellaCacheArb.io.mem
  outer.dcache.module.io.lsu <> lsu.io.dmem

  // Generate a descriptive string
  val frontendStr = outer.frontend.module.toString
  val coreStr = core.toString
  val boomTileStr =
    (BoomCoreStringPrefix(s"======BOOM Tile ${staticIdForMetadataUseOnly} Params======") + "\n"
    + frontendStr
    + coreStr + "\n")

  override def toString: String = boomTileStr

  print(boomTileStr)
}
