// See LICENSE.SiFive for license details.

package uncore.devices

import Chisel._
import junctions._
import util._
import regmapper._
import tile.XLen
import uncore.tilelink2._
import config._


/** Constant values used by both Debug Bus Response & Request
  */

object DMIConsts{

  def dmiDataSize = 32

  def dmiOpSize = 2
  def dmi_OP_NONE            = "b00".U
  def dmi_OP_READ            = "b01".U
  def dmi_OP_WRITE           = "b10".U

  def dmiRespSize = 2
  def dmi_RESP_SUCCESS     = "b00".U
  def dmi_RESP_FAILURE     = "b01".U
  def dmi_RESP_HW_FAILURE  = "b10".U
  // This is used outside this block
  // to indicate 'busy'.
  def dmi_RESP_RESERVED    = "b11".U

  def dmi_haltStatusAddr   = 0x40

}

object DsbBusConsts {

  def sbAddrWidth = 12
  def sbIdWidth   = 10 

}

object DsbRegAddrs{

  // These may need to move around to be used by the serial interface.

  // These are used by the ROM.
  def HALTED       = 0x100
  def GOING        = 0x104
  def RESUMING     = 0x108
  def EXCEPTION    = 0x10C

  def GO           = 0x400

  def ROMBASE      = 0x800
  def ENTRY        = 0x800
  //def EXCEPTION    = 0x808
  def RESUME       = 0x804

  def WHERETO      = 0x300
  def ABSTRACT     = 0x304
  def PROGBUF      = 0x340

  // This shows up in HartInfo
  def DATA         = 0x380

  //Not implemented: Serial.
 
}

/** Enumerations used both in the hardware
  * and in the configuration specification.
  */

object DebugModuleAccessType extends scala.Enumeration {
  type DebugModuleAccessType = Value
  val Access8Bit, Access16Bit, Access32Bit, Access64Bit, Access128Bit = Value
}
import DebugModuleAccessType._

object DebugModuleHartStatus extends scala.Enumeration {
  type DebugModuleHartStatus = Value
  val Halted, Running, Unavailable, NonExistent = Value
}
import DebugModuleHartStatus._

object DebugAbstractCommandError extends scala.Enumeration {
  type DebugAbstractCommandError = Value
  val None, ErrBusy, ErrNotSupported, ErrException, ErrHaltResume = Value
}
import DebugAbstractCommandError._


object DebugAbstractCommandType extends scala.Enumeration {
  type DebugAbstractCommandType = Value
  val AccessRegister, QuickAccess  = Value
}
import DebugAbstractCommandType._



/** Parameters exposed to the top-level design, set based on
  * external requirements, etc.
  *
  *  This object checks that the parameters conform to the 
  *  full specification. The implementation which receives this
  *  object can perform more checks on what that implementation
  *  actually supports.
  *  nComponents : The number of components to support debugging.
  *  nDMIAddrSize : Size of the Debug Bus Address
  *  nAbstractDataWords : Number of 32-bit words for Abstract Commands
  *  nProgamBufferWords : Number of 32-bit words for Program Buffer
  *  hasBusMaster: Whethr or not a bus master should be included
  *    The size of the accesses supported by the Bus Master. 
  *  nSerialPorts : Number of serial ports to instantiate
  *  supportQuickAccess : Whether or not to support the quick access command.
  *  supportHartArray : Whether or not to implement the hart array register.
  **/


case class DebugModuleConfig (
  nComponents   : Int,
  nDMIAddrSize  : Int,
  nProgramBufferWords: Int,
  nAbstractDataWords : Int,
  hasBusMaster : Boolean,
  hasAccess128 : Boolean,
  hasAccess64  : Boolean,
  hasAccess32  : Boolean,
  hasAccess16  : Boolean,
  hasAccess8   : Boolean,
  nSerialPorts : Int,
  supportQuickAccess : Boolean,
  supportHartArray : Boolean
    //TODO  configStringAddr : Int
) {

  if (hasBusMaster == false){
    require (hasAccess128 == false)
    require (hasAccess64  == false)
    require (hasAccess32  == false)
    require (hasAccess16  == false)
    require (hasAccess8   == false)
  }

  require (nSerialPorts <= 8)

  require ((nDMIAddrSize >= 7) && (nDMIAddrSize <= 32))

  private val maxComponents = 1024
  require (nComponents > 0 && nComponents <= maxComponents)

  // Note there are only 4 bits to represent these values.
  require ((nAbstractDataWords  > 0)  && (nAbstractDataWords  < 16))
  require ((nProgramBufferWords >= 0) && (nProgramBufferWords < 16))

  if (supportQuickAccess) {
    // TODO: Check that quick access requirements are met.
  }
}

class DefaultDebugModuleConfig (val ncomponents : Int, val xlen:Int /*TODO , val configStringAddr: Int*/)
    extends DebugModuleConfig(
      nComponents = ncomponents,
      nDMIAddrSize = 7,
      //TODO: what should this value be
      nProgramBufferWords   =  8,
      nAbstractDataWords = xlen match{
        case 32  => 1 
        case 64  => 2
        case 128 => 4
      },
      hasBusMaster = false,
      hasAccess128 = false, 
      hasAccess64 = false, 
      hasAccess32 = false, 
      hasAccess16 = false, 
      hasAccess8 = false, 
      nSerialPorts = 0,
      supportQuickAccess = false,
      supportHartArray = false
        // TODO configStringAddr = configStringAddr
        // TODO: accept a mapping function from HARTID -> HARTSEL
    )

case object DMKey extends Field[DebugModuleConfig]

// *****************************************
// Module Interfaces
// 
// *****************************************

/** Structure to define the contents of a Debug Bus Request
  */

class DMIReq(addrBits : Int) extends Bundle {
  val addr = UInt(addrBits.W)
  val data = UInt(DMIConsts.dmiDataSize.W)
  val op   = UInt(DMIConsts.dmiOpSize.W)

  override def cloneType = new DMIReq(addrBits).asInstanceOf[this.type]
}

/** Structure to define the contents of a Debug Bus Response
  */
class DMIResp( ) extends Bundle {
  val data = UInt(DMIConsts.dmiDataSize.W)
  val resp = UInt(DMIConsts.dmiRespSize.W)

}

/** Structure to define the top-level DMI interface 
  *  of DebugModule.
  *  DebugModule is the consumer of this interface.
  *  Therefore it has the 'flipped' version of this.
  */

class DMIIO(implicit val p: Parameters) extends ParameterizedBundle()(p) {
  val req = new  DecoupledIO(new DMIReq(p(DMKey).nDMIAddrSize))
  val resp = new DecoupledIO(new DMIResp).flip()
}

trait HasDebugModuleParameters {
  implicit val p: Parameters
  val cfg = p(DMKey)
}

/** Debug Module I/O, with the exclusion of the RegisterRouter
  *  Access interface.
  */

trait DebugModuleBundle extends Bundle with HasDebugModuleParameters {
  val dmi = new DMIIO()(p).flip()
  val debugInterrupts = Vec(cfg.nComponents, Bool()).asOutput
  val debugUnavail    = Vec(cfg.nComponents, Bool()).asInput
  val ndreset         = Bool(OUTPUT)
  val debugActive     = Bool(OUTPUT)
}

// *****************************************
// The Module 
// 
// *****************************************

/** Parameterized version of the Debug Module defined in the
  *  RISC-V Debug Specification 
  *  
  *  DebugModule is a slave to two masters:
  *    The Debug Bus (DMI) -- implemented as a generic Decoupled IO with request
  *                           and response channels
  *    The System Bus -- implemented as generic RegisterRouter
  *  
  *  DebugModule is responsible for holding registers, RAM, and ROM
  *      to support debug interactions, as well as driving interrupts
  *      to a configurable number of components in the system.
  *      It must also maintain a state machine to track the state of harts
  *      being debugged.
  *      It is also responsible for some reset lines.
  */

trait DebugModule extends Module with HasDebugModuleParameters with HasRegMap {

  val io: DebugModuleBundle

  //--------------------------------------------------------------
  // Import constants for shorter variable names
  //--------------------------------------------------------------

  import DMI_RegAddrs._
  import DsbRegAddrs._
  import DsbBusConsts._
  import DMIConsts._

  //--------------------------------------------------------------
  // Sanity Check Configuration For this implementation.
  //--------------------------------------------------------------

  require (cfg.nSerialPorts == 0)
  require (cfg.hasBusMaster == false)
  require (cfg.supportQuickAccess == false)
  require (cfg.supportHartArray == false)

  //--------------------------------------------------------------
  // Register & Wire Declarations
  //--------------------------------------------------------------

  val debugIntRegs = Reg(init=Vec.fill(cfg.nComponents){Bool(false)})
  val haltedBitRegs  = Reg(init=Vec.fill(cfg.nComponents){Bool(false)})

  val dmiReq    = io.dmi.req.bits
  val dmiWrEn   = Wire(Bool())
  val dmiRdEn   = Wire(Bool())

  // --- regmapper outputs

  val hartHaltedWrEn       = Wire(Bool())
  val hartHaltedId         = Wire(UInt(sbIdWidth.W))
  val hartGoingWrEn        = Wire(Bool())
  val hartGoingId          = Wire(UInt(sbIdWidth.W))
  val hartResumingWrEn     = Wire(Bool())
  val hartResumingId       = Wire(UInt(sbIdWidth.W))
  val hartExceptionWrEn    = Wire(Bool())
  val hartExceptionId      = Wire(UInt(sbIdWidth.W))

  //--------------------------------------------------------------
  // DMI Registers 
  //--------------------------------------------------------------


  //----DMCONTROL

  //TODO: Hart Array version of this.
  val DMCONTROLReset = Wire(init = (new DMCONTROLFields()).fromBits(0.U))
  val DMCONTROLReg = RegInit(DMCONTROLReset)

  val DMCONTROLRdData = Wire(init = DMCONTROLReg)

  val DMCONTROLWrData = (new DMCONTROLFields()).fromBits(dmiReq.data)
  val DMCONTROLWrEn   = dmiWrEn & (dmiReq.addr === DMI_DMCONTROL)

  val dmactive = DMCONTROLReg.dmactive

  when (~dmactive) {
    DMCONTROLReg := DMCONTROLReset
  } .otherwise {
    when (DMCONTROLWrEn) {
      DMCONTROLReg.ndmreset  := DMCONTROLWrData.ndmreset
      DMCONTROLReg.haltreq   := DMCONTROLWrData.haltreq
      DMCONTROLReg.resumereq := DMCONTROLWrData.resumereq
      DMCONTROLReg.hartsel   := DMCONTROLWrData.hartsel
    }
  }

  // Put this last to override its own effects.
  when (DMCONTROLWrEn) {
    DMCONTROLReg.dmactive := DMCONTROLWrData.dmactive
  }

  //----DMSTATUS

  val DMSTATUSRdData = Wire(init = (new DMSTATUSFields()).fromBits(0.U))
  DMSTATUSRdData.authenticated := true.B // Not implemented
  DMSTATUSRdData.version       := 1.U

  val unavailVec = Wire(Vec(cfg.nComponents, Bool()))
  unavailVec := io.debugUnavail

  // TODO: Hart Array Version of this
  when (DMCONTROLReg.hartsel >= cfg.nComponents.U) {
    DMSTATUSRdData.anynonexistent := true.B
    DMSTATUSRdData.allnonexistent := true.B
  } .elsewhen (haltedBitRegs(DMCONTROLReg.hartsel)) {
    DMSTATUSRdData.anyhalted := true.B
    DMSTATUSRdData.allhalted := true.B
  } .elsewhen(unavailVec(DMCONTROLReg.hartsel)) {
    DMSTATUSRdData.anyunavail := true.B
    DMSTATUSRdData.allunavail := true.B
  } .otherwise {
    DMSTATUSRdData.anyrunning := true.B
    DMSTATUSRdData.allrunning := true.B
  }


  //----HARTINFO

  val HARTINFORdData = Wire (init = (new HARTINFOFields()).fromBits(0.U))
  HARTINFORdData.dataaccess  := true.B
  HARTINFORdData.datasize    := cfg.nAbstractDataWords.U
  HARTINFORdData.dataaddr    := DsbRegAddrs.DATA.U

  //----HALTSUM (and halted registers)
  val numHaltedStatus = ((cfg.nComponents - 1) / 32) + 1

  val haltedStatus   = Wire(Vec(numHaltedStatus, Bits(width = 32)))

  for (ii <- 0 until numHaltedStatus) {
    haltedStatus(ii) := Cat(haltedBitRegs.slice(ii * 32, (ii + 1) * 32).reverse)
  }

  require ((DMIConsts.dmi_haltStatusAddr & 0xF) == 0)
  val dmiHaltedStatusIdx      = dmiReq.addr & 0xF.U(4.W)
  val dmiHaltedStatusIdxValid = ((dmiReq.addr & ~(0xF.U(cfg.nDMIAddrSize.W))) === DMIConsts.dmi_haltStatusAddr.U) && (dmiHaltedStatusIdx < cfg.nComponents.U)

  val haltedStatusRdData = haltedStatus(dmiHaltedStatusIdx)

  val haltedSummary = Cat(haltedStatus.map(_.orR).reverse)

  val HALTSUMRdData = (new HALTSUMFields()).fromBits(haltedSummary)

  //----ABSTRACTCS

  val ABSTRACTCSReset = Wire(init = (new ABSTRACTCSFields()).fromBits(0.U))
  ABSTRACTCSReset.datacount := cfg.nAbstractDataWords.U
  ABSTRACTCSReset.progsize  := cfg.nProgramBufferWords.U


  val ABSTRACTCSReg = RegInit(ABSTRACTCSReset)
  val ABSTRACTCSWrData = (new ABSTRACTCSFields()).fromBits(dmiReq.data)
  val ABSTRACTCSRdData = Wire(init = ABSTRACTCSReg)

  val ABSTRACTCSWrEnMaybe = dmiWrEn & (dmiReq.addr === DMI_ABSTRACTCS)

  val ABSTRACTCSWrEnLegal = Wire(init = false.B)
  val ABSTRACTCSWrEn      = ABSTRACTCSWrEnMaybe && ABSTRACTCSWrEnLegal

  val errorBusy        = Wire(init = false.B)
  val errorException   = Wire(init = false.B)
  val errorUnsupported = Wire(init = false.B)
  val errorHaltResume  = Wire(init = false.B)

  val abstractCSClearError = ABSTRACTCSWrEn && (ABSTRACTCSWrData.cmderr === 0.U)

  when(~dmactive){
    ABSTRACTCSReg := ABSTRACTCSReset
  }.otherwise {

    when (errorBusy){
      ABSTRACTCSReg.cmderr := DebugAbstractCommandError.ErrBusy.id.U
    }.elsewhen (errorException) {
      ABSTRACTCSReg.cmderr := DebugAbstractCommandError.ErrException.id.U
    }.elsewhen (errorUnsupported) {
      ABSTRACTCSReg.cmderr := DebugAbstractCommandError.ErrNotSupported.id.U
    }.elsewhen (errorHaltResume) {
      ABSTRACTCSReg.cmderr := DebugAbstractCommandError.ErrHaltResume.id.U
    }.otherwise {
      when (abstractCSClearError){
        ABSTRACTCSReg.cmderr := DebugAbstractCommandError.None.id.U
      }
    }

    when (ABSTRACTCSWrEn) {
      if (cfg.nAbstractDataWords > 0) ABSTRACTCSReg.autoexec0 := ABSTRACTCSWrData.autoexec0
      if (cfg.nAbstractDataWords > 1) ABSTRACTCSReg.autoexec1 := ABSTRACTCSWrData.autoexec1
      if (cfg.nAbstractDataWords > 2) ABSTRACTCSReg.autoexec2 := ABSTRACTCSWrData.autoexec2
      if (cfg.nAbstractDataWords > 3) ABSTRACTCSReg.autoexec3 := ABSTRACTCSWrData.autoexec3
      if (cfg.nAbstractDataWords > 4) ABSTRACTCSReg.autoexec4 := ABSTRACTCSWrData.autoexec4
      if (cfg.nAbstractDataWords > 5) ABSTRACTCSReg.autoexec5 := ABSTRACTCSWrData.autoexec5
      if (cfg.nAbstractDataWords > 6) ABSTRACTCSReg.autoexec6 := ABSTRACTCSWrData.autoexec6
      if (cfg.nAbstractDataWords > 7) ABSTRACTCSReg.autoexec7 := ABSTRACTCSWrData.autoexec7
    }
  }

  // For busy, see below state machine.
  val abstractCommandBusy = Wire(init = true.B)
  ABSTRACTCSRdData.busy := abstractCommandBusy

  //---- COMMAND

  val COMMANDReset = (new COMMANDFields()).fromBits(0.U)
  val COMMANDReg = RegInit(COMMANDReset)

  val COMMANDWrData       = (new COMMANDFields()).fromBits(dmiReq.data)
  val COMMANDWrEnMaybe    = dmiWrEn & (dmiReq.addr === DMI_COMMAND)
  val COMMANDWrEnLegal    = Wire(init = false.B)

  val COMMANDWrEn = COMMANDWrEnMaybe && COMMANDWrEnLegal
  val COMMANDRdData = COMMANDReg

  when (~dmactive) {
    COMMANDReg := COMMANDReset
  }.otherwise {
    when (COMMANDWrEn) {
      COMMANDReg := COMMANDWrData
    }
  }


  // --- Abstract Data

  val abstractDataWidth     = 32
  val abstractDataAddrWidth = log2Up(cfg.nAbstractDataWords)

  // These are byte addressible, s.t. the Processor can use
  // byte-addressible instructions to store to them.
  val abstractDataMem       = RegInit(Vec.fill(cfg.nAbstractDataWords*4){0.U(8.W)})

  val dmiAbstractDataIdx        = Wire(UInt(abstractDataAddrWidth.W))
  val dmiAbstractDataIdxValid   = Wire(Bool())
  val dmiAbstractDataRdData      = Wire (UInt(32.W))
  val dmiAbstractDataWrData      = Wire(UInt(32.W))

  val dmiAbstractDataOffset = log2Up(abstractDataWidth/8)

  // --- Program Buffer


  val programBufferDataWidth  = 32
  val programBufferAddrWidth = log2Up(cfg.nProgramBufferWords)

  val programBufferMem    = RegInit(Vec.fill(cfg.nProgramBufferWords*4){0.U(8.W)})

  val dmiProgramBufferIdx   = Wire(UInt(programBufferAddrWidth.W))
  val dmiProgramBufferIdxValid   = Wire(Bool())
  val dmiProgramBufferRdData = Wire (UInt(32.W))
  val dmiProgramBufferWrData = Wire(UInt(32.W))

  val dmiProgramBufferOffset = log2Up(programBufferDataWidth/8)


  //--------------------------------------------------------------
  // Interrupt Registers
  //--------------------------------------------------------------
  
  for (component <- 0 until cfg.nComponents) {
    io.debugInterrupts(component) := debugIntRegs(component)
  }

  // Halt request registers are written by write to DMCONTROL.haltreq
  // and cleared by writes to DMCONTROL.resumereq.
  
  for (component <- 0 until cfg.nComponents) {
    when (~dmactive) {
      debugIntRegs(component) := false.B
    }. otherwise {
      when (DMCONTROLWrEn) {
        when (DMCONTROLWrData.hartsel === component.U) {
          debugIntRegs(component) := (debugIntRegs(component) | DMCONTROLWrData.haltreq) &
          ~(DMCONTROLWrData.resumereq)
        }
      }
    }
  }

  //--------------------------------------------------------------
  // These bits are implementation-specific bits set
  // by harts executing code.
  //--------------------------------------------------------------

  for (component <- 0 until cfg.nComponents) {
    when (~dmactive) {
      haltedBitRegs(component) := false.B
    }.otherwise {
      when (hartHaltedWrEn) {
        when (hartHaltedId === component.U) {
          haltedBitRegs(component) := true.B
        }
      }.elsewhen (hartResumingWrEn) {
        when (hartResumingId === component.U) {
          haltedBitRegs(component) := false.B
        }
      }
    }
  }

 
  //--------------------------------------------------------------
  // Abstract Data Access (Debug Bus ... System Bus can override)
  //--------------------------------------------------------------

  dmiAbstractDataIdx       := dmiReq.addr & 0xF.U(4.W)
  dmiAbstractDataIdxValid  := ((dmiReq.addr & ~(0xF.U(cfg.nDMIAddrSize.W))) === DMI_DATA0) && (dmiAbstractDataIdx < cfg.nAbstractDataWords.U)

  val dmiAbstractDataWrEn  = dmiAbstractDataIdxValid && dmiWrEn
  val dmiAbstractDataRdEn  = dmiAbstractDataIdxValid && dmiRdEn

  val dmiAbstractDataFields = List.tabulate(cfg.nAbstractDataWords) { ii =>
    val slice = abstractDataMem.slice(ii * 4, (ii+1)*4)
    slice.reduce[UInt]{ case (x: UInt, y: UInt) => Cat(y, x) }
  }

  when (dmiWrEn & dmiAbstractDataIdxValid) {
    for (ii <- 0 until 4) {
      abstractDataMem((dmiAbstractDataIdx << 2) + ii.U) := dmiReq.data((8*(ii+1)-1), (8*ii))
    }
  }

  dmiAbstractDataRdData := dmiAbstractDataFields(dmiAbstractDataIdx)

  //--------------------------------------------------------------
  // Program Buffer Access (Debug Bus ... System Bus can override)
  //--------------------------------------------------------------

  dmiProgramBufferIdx       := dmiReq.addr & 0xF.U(4.W)
  dmiProgramBufferIdxValid  := ((dmiReq.addr & ~(0xF.U(cfg.nDMIAddrSize.W))) === DMI_PROGBUF0) && (dmiProgramBufferIdx < cfg.nProgramBufferWords.U)

  val dmiProgramBufferFields = List.tabulate(cfg.nProgramBufferWords) { ii =>
    val slice = programBufferMem.slice(ii * 4, (ii+1)*4)
    slice.reduce[UInt]{ case (x: UInt, y: UInt) => Cat(y, x)}
  }

  dmiProgramBufferRdData := dmiProgramBufferFields(dmiProgramBufferIdx)

  when (dmiWrEn & dmiProgramBufferIdxValid) {
    for (ii <- 0 until 4) {
      programBufferMem((dmiProgramBufferIdx << 2) + ii.U) := dmiReq.data((8*(ii+1)-1), (8*ii))
    }
  }

  //--------------------------------------------------------------
  // DMI Access
  //--------------------------------------------------------------

  val dmiBusyReg = RegInit(false.B)

  // -----------------------------------------
  // DMI Access Read Mux
  val dmiRdData = Wire(UInt(DMIConsts.dmiDataSize.W))

  when     (dmiReq.addr === DMI_DMCONTROL)     {dmiRdData := DMCONTROLRdData.asUInt()}
    .elsewhen (dmiReq.addr === DMI_DMSTATUS)   {dmiRdData := DMSTATUSRdData.asUInt()}
    .elsewhen (dmiReq.addr === DMI_HARTINFO)   {dmiRdData := HARTINFORdData.asUInt()}
    .elsewhen (dmiReq.addr === DMI_HALTSUM)    {dmiRdData := HALTSUMRdData.asUInt()}
    .elsewhen (dmiReq.addr === DMI_ABSTRACTCS) {dmiRdData := ABSTRACTCSRdData.asUInt()}
    .elsewhen (dmiReq.addr === DMI_COMMAND)    {dmiRdData := COMMANDRdData.asUInt()}
    .elsewhen (dmiAbstractDataIdxValid)        {dmiRdData := dmiAbstractDataRdData}
    .elsewhen (dmiProgramBufferIdxValid)       {dmiRdData := dmiProgramBufferRdData}
    .elsewhen (dmiHaltedStatusIdxValid)        {dmiRdData := haltedStatusRdData}
    .otherwise {dmiRdData := 0.U}

  // There is no way to return failure without SB or Serial, which are not
  // implemented yet.


  // -----------------------------------------
  // DMI Access State Machine Decode (Combo)

  val dmiResult  = Wire(new DMIResp())

  dmiResult.resp := dmi_RESP_SUCCESS
  dmiResult.data := dmiRdData

  val dmiRespReg = Reg(new DMIResp())

  io.dmi.req.ready := (!dmiBusyReg) || (dmiBusyReg && io.dmi.resp.fire())

  io.dmi.resp.valid := dmiBusyReg
  io.dmi.resp.bits  := dmiRespReg

  dmiWrEn := (dmiReq.op === dmi_OP_WRITE) && io.dmi.req.fire()
  dmiRdEn := (dmiReq.op === dmi_OP_READ)  && io.dmi.req.fire()

  // -----------------------------------------
  // DMI Access State Machine Update (Seq)

  when (!dmiBusyReg) {
    when (io.dmi.req.fire()){
      dmiBusyReg := true.B
      dmiRespReg := dmiResult
    }
  } .otherwise {
    when (io.dmi.req.fire()){
      dmiBusyReg := true.B
      dmiRespReg := dmiResult
    }.elsewhen (io.dmi.resp.fire()){
      dmiBusyReg := false.B
    }
  }

  //--------------------------------------------------------------
  // Debug ROM
  //--------------------------------------------------------------

  val romRegFields  =  DebugRomContents().map( x => RegField.r(8, (x.toInt & 0xFF).U))

  //--------------------------------------------------------------
  // "Variable" ROM Generation
  //--------------------------------------------------------------

  val goProgramBuffer = Wire(init = false.B)
  val goResume        = Wire(init = false.B)
  val goAbstract      = Wire(init = false.B)

  val whereToReg = RegInit(0.U(32.W))

  val jalProgBuf  = Wire(init = (new GeneratedUJ()).fromBits(rocket.Instructions.JAL.value.U))
  jalProgBuf.setImm(PROGBUF - WHERETO)
  jalProgBuf.rd := 0.U

  val jalAbstract  = Wire(init = (new GeneratedUJ()).fromBits(rocket.Instructions.JAL.value.U))
  jalAbstract.setImm(ABSTRACT - WHERETO)
  jalProgBuf.rd := 0.U

  val jalResume  = Wire(init = (new GeneratedUJ()).fromBits(rocket.Instructions.JAL.value.U))
  jalResume.setImm(RESUME - WHERETO)
  jalResume.rd := 0.U

  when (goProgramBuffer) {
    whereToReg := jalProgBuf.asUInt()
  }.elsewhen (goResume) {
    whereToReg := jalResume.asUInt()
  }.elsewhen (goAbstract) {
    whereToReg := jalAbstract.asUInt()
  }

  val goReg            = Reg (init = false.B)
  when (goProgramBuffer | goResume | goAbstract) {
    goReg := true.B
  }.elsewhen (hartGoingWrEn){
    assert(hartGoingId === 0.U, "Unexpected 'GOING' hart: %x, expected %x", hartGoingId, DMCONTROLReg.hartsel)
    goReg := false.B
  }

  val goBytes = Wire(init = Vec.fill(cfg.nComponents){0.U(8.W)})
  goBytes(DMCONTROLReg.hartsel) := Cat(0.U(7.W), goReg)

  //----------------------------
  // Abstract Command Decoding & Generation
  //----------------------------

  val accessRegisterCommandWr  = Wire(init = (new ACCESS_REGISTERFields()).fromBits(COMMANDWrData.asUInt()))
  val accessRegisterCommandReg = Wire(init = (new ACCESS_REGISTERFields()).fromBits(COMMANDReg.asUInt()))

  // TODO: (maybe) Quick Access
  // TODO: Config String
 
  class GeneratedI extends Bundle {
    val imm    = UInt(12.W)
    val rs1    = UInt(5.W)
    val funct3 = UInt(3.W)
    val rd     = UInt(5.W)
    val opcode = UInt(7.W)
  }

  class GeneratedS extends Bundle {
    val immhi  = UInt(7.W)
    val rs2    = UInt(5.W)
    val rs1    = UInt(5.W)
    val funct3 = UInt(3.W)
    val immlo  = UInt(5.W)
    val opcode = UInt(7.W)
  }

  class GeneratedUJ extends Bundle {
    val imm3    = UInt(1.W)
    val imm0    = UInt(10.W)
    val imm1    = UInt(1.W)
    val imm2    = UInt(8.W)
    val rd      = UInt(5.W)
    val opcode  = UInt(7.W)

    def setImm(imm: Int) : Unit = {
      // TODO: Check bounds of imm.

      require(imm % 2 == 0, "Immediate must be even for UJ encoding.")
      val immWire = Wire(init = imm.S(21.W))
      val immBits = Wire(init = Vec(immWire.toBools))

      imm0 := immBits.slice(1,  1  + 10).asUInt()
      imm1 := immBits.slice(11, 11 + 11).asUInt()
      imm2 := immBits.slice(12, 12 + 8).asUInt()
      imm3 := immBits.slice(20, 20 + 1).asUInt()
    }
  }

  val abstractGeneratedReg = Reg(UInt(32.W))
  val abstractGeneratedI = Wire(new GeneratedI())
  val abstractGeneratedS = Wire(new GeneratedS())

  abstractGeneratedI.opcode := ((new GeneratedI()).fromBits(rocket.Instructions.LW.value.U)).opcode
  abstractGeneratedI.rd     := (accessRegisterCommandReg.regno & 0x1F.U)
  abstractGeneratedI.funct3 := accessRegisterCommandReg.size
  abstractGeneratedI.rs1    := 0.U
  abstractGeneratedI.imm    := DATA.U

  abstractGeneratedS.opcode := ((new GeneratedI()).fromBits(rocket.Instructions.SW.value.U)).opcode
  abstractGeneratedS.immlo  := (DATA & 0x1F).U
  abstractGeneratedS.funct3 := accessRegisterCommandReg.size
  abstractGeneratedS.rs1    := 0.U
  abstractGeneratedS.rs2    := (accessRegisterCommandReg.regno & 0x1F.U)
  abstractGeneratedS.immhi  := (DATA >> 5).U

  when (goAbstract) {
    when (accessRegisterCommandReg.write) {
      // To write a register, we need to do LW.
      abstractGeneratedReg := abstractGeneratedI.asUInt()
    }.otherwise {
      // To read a register, we need to do SW.
      abstractGeneratedReg := abstractGeneratedS.asUInt()
    }
  }

  val ebreakInstruction = Wire(init = rocket.Instructions.EBREAK.value.U)

  //--------------------------------------------------------------
  // System Bus Access
  //--------------------------------------------------------------

  // Local reg mapper function : Notify when written, but give the value as well.
  def wValue (n: Int, value: UInt, set: Bool) : RegField = {
    RegField(n, value, RegWriteFn((valid, data) => {set := valid ; value := data; Bool(true)}))
  }

  regmap(
    // This memory is writable.
    HALTED      -> Seq(wValue(sbIdWidth, hartHaltedId, hartHaltedWrEn)),
    GOING       -> Seq(wValue(sbIdWidth, hartGoingId,  hartGoingWrEn)),
    RESUMING    -> Seq(wValue(sbIdWidth, hartResumingId,  hartResumingWrEn)),
    EXCEPTION   -> Seq(wValue(sbIdWidth, hartExceptionId,  hartExceptionWrEn)),
    DATA        -> abstractDataMem.map(x => RegField(8, x)),
    PROGBUF     -> programBufferMem.map(x => RegField(8, x)),

    // These sections are read-only.
    ROMBASE     -> romRegFields,
    GO          -> goBytes.map(x => RegField.r(8, x)),   
    WHERETO     -> Seq(RegField.r(32, whereToReg)),
    ABSTRACT    -> Seq(RegField.r(32, abstractGeneratedReg), RegField.r(32, ebreakInstruction))
  )

  //--------------------------------------------------------------
  // Abstract Command State Machine
  //--------------------------------------------------------------

  object CtrlState extends scala.Enumeration {
    type CtrlState = Value
    val Waiting, CheckGenerate, PreExec, Abstract, PostExec = Value

    def apply( t : Value) : UInt = {
      t.id.U(log2Up(values.size).W)
    }
  }
  import CtrlState._

  val ctrlStateReg = RegInit(CtrlState(Waiting))

  // Combo
  val hartHalted   = DMSTATUSRdData.allhalted
  val ctrlStateNxt = Wire(init = ctrlStateReg)
  val autoexecVec  = Wire(init = Vec.fill(8){false.B})

  val dmiAbstractDataAccess = dmiAbstractDataWrEn | dmiAbstractDataRdEn

  if (cfg.nAbstractDataWords > 0) autoexecVec(0) := (dmiAbstractDataIdx === 0.U) && dmiAbstractDataAccess && ABSTRACTCSReg.autoexec0
  if (cfg.nAbstractDataWords > 1) autoexecVec(1) := (dmiAbstractDataIdx === 1.U) && dmiAbstractDataAccess && ABSTRACTCSReg.autoexec1
  if (cfg.nAbstractDataWords > 2) autoexecVec(2) := (dmiAbstractDataIdx === 2.U) && dmiAbstractDataAccess && ABSTRACTCSReg.autoexec2
  if (cfg.nAbstractDataWords > 3) autoexecVec(3) := (dmiAbstractDataIdx === 3.U) && dmiAbstractDataAccess && ABSTRACTCSReg.autoexec3
  if (cfg.nAbstractDataWords > 4) autoexecVec(4) := (dmiAbstractDataIdx === 4.U) && dmiAbstractDataAccess && ABSTRACTCSReg.autoexec4
  if (cfg.nAbstractDataWords > 5) autoexecVec(5) := (dmiAbstractDataIdx === 5.U) && dmiAbstractDataAccess && ABSTRACTCSReg.autoexec5
  if (cfg.nAbstractDataWords > 6) autoexecVec(6) := (dmiAbstractDataIdx === 6.U) && dmiAbstractDataAccess && ABSTRACTCSReg.autoexec6
  if (cfg.nAbstractDataWords > 7) autoexecVec(7) := (dmiAbstractDataIdx === 7.U) && dmiAbstractDataAccess && ABSTRACTCSReg.autoexec7

  val autoexec = autoexecVec.reduce(_ || _)

  //------------------------

  // DMI Register Control and Status
  abstractCommandBusy := (ctrlStateReg != CtrlState(Waiting))

  ABSTRACTCSWrEnLegal := (ctrlStateReg === CtrlState(Waiting))
  COMMANDWrEnLegal    := (ctrlStateReg === CtrlState(Waiting))

  errorBusy := (ABSTRACTCSWrEnMaybe && ~ABSTRACTCSWrEnLegal) ||
               (COMMANDWrEnMaybe && ~COMMANDWrEnLegal)


  // TODO: Config String Command, maybe Quick Access
  val commandWrIsAccessRegister = (COMMANDWrData.cmdtype === DebugAbstractCommandType.AccessRegister.id.U)
  val commandRegIsAccessRegister = (COMMANDReg.cmdtype === DebugAbstractCommandType.AccessRegister.id.U)

  val commandRegIsUnsupported = Wire(init = true.B)
  val commandRegBadHaltResume = Wire(init = false.B)
  when (commandRegIsAccessRegister) {
    when ((accessRegisterCommandReg.regno >= 0x1000.U && accessRegisterCommandReg.regno <= 0x101F.U)){
      commandRegIsUnsupported := false.B
      commandRegBadHaltResume := ~hartHalted
    }
  }

  val wrAccessRegisterCommand  = COMMANDWrEn && commandWrIsAccessRegister
  val regAccessRegisterCommand = autoexec    && commandRegIsAccessRegister
  //------------------------
  // Variable ROM STATE MACHINE
  // -----------------------

  when (ctrlStateReg === CtrlState(Waiting)){

    when (wrAccessRegisterCommand || regAccessRegisterCommand) {
      ctrlStateNxt := CtrlState(CheckGenerate)
    }.elsewhen(DMCONTROLWrEn && DMCONTROLWrData.resumereq) {
      goResume := true.B
    }

  }.elsewhen (ctrlStateReg === CtrlState(CheckGenerate)){

    // We use this state to ensure that the COMMAND has been
    // registered by the time that we need to use it, to avoid
    // generating it directly from the COMMANDWrData.

    when (commandRegIsUnsupported) {
      errorUnsupported := true.B
      ctrlStateNxt := CtrlState(Waiting)
    }.elsewhen (commandRegBadHaltResume){
      errorHaltResume := true.B
      ctrlStateNxt := CtrlState(Waiting)
    }.otherwise {
      when (accessRegisterCommandReg.preexec) {
        ctrlStateNxt    := CtrlState(PreExec)
        goProgramBuffer := true.B
      }.otherwise {
        ctrlStateNxt := CtrlState(Abstract)
        goAbstract := true.B
      }
    }
    }.elsewhen (ctrlStateReg === CtrlState(PreExec)) {

    // We can't just look at 'hartHalted' here, because
    // hartHaltedWrEn is overloaded to mean 'got an ebreak'
    // which may have happened when we were already halted.
    when(goReg === false.B && hartHaltedWrEn && (hartHaltedId === DMCONTROLReg.hartsel)){
      ctrlStateNxt := CtrlState(Abstract)
      goAbstract := true.B
    }
    when(hartExceptionWrEn) {
      assert(hartExceptionId === 0.U,  "Unexpected 'EXCEPTION' hart, %x, expected %x", hartExceptionId, DMCONTROLReg.hartsel)
      ctrlStateNxt := CtrlState(Waiting)
      errorException := true.B
    }
  }.elsewhen (ctrlStateReg === CtrlState(Abstract)) {
  
    // We can't just look at 'hartHalted' here, because
    // hartHaltedWrEn is overloaded to mean 'got an ebreak'
    // which may have happened when we were already halted.
    when(goReg === false.B && hartHaltedWrEn && (hartHaltedId === DMCONTROLReg.hartsel)){
      when (accessRegisterCommandReg.postexec) {
        ctrlStateNxt := CtrlState(PostExec)
        goProgramBuffer := true.B
      }.otherwise {
        ctrlStateNxt := CtrlState(Waiting)
      }
    }

    when(hartExceptionWrEn) {
      assert(hartExceptionId === 0.U, "Unexpected 'EXCEPTION' hart, %x, expected %x", hartExceptionId, DMCONTROLReg.hartsel)
      ctrlStateNxt := CtrlState(Waiting)
      errorUnsupported := true.B
    }

  }.elsewhen (ctrlStateReg === CtrlState(PostExec)) {

    // We can't just look at 'hartHalted' here, because
    // hartHaltedWrEn is overloaded to mean 'got an ebreak'
    // which may have happened when we were already halted.
    when(goReg === false.B && hartHaltedWrEn && (hartHaltedId === DMCONTROLReg.hartsel)){
      ctrlStateNxt := CtrlState(Waiting)
    }

    when(hartExceptionWrEn) {
      assert(hartExceptionId === 0.U, "Unexpected 'EXCEPTION' hart, %x, expected %x", hartExceptionId, DMCONTROLReg.hartsel)
      ctrlStateNxt := CtrlState(Waiting)
      errorException := true.B
    }
  }

  // Sequential
  when (!dmactive) {
    ctrlStateReg := CtrlState(Waiting)
  }.otherwise {
    ctrlStateReg := ctrlStateNxt
  }

  //--------------------------------------------------------------
  // Misc. Outputs
  //--------------------------------------------------------------

  io.ndreset   := DMCONTROLReg.ndmreset

  io.debugActive := dmactive

}

/** Create a concrete TL2 Slave for the DebugModule RegMapper interface.
  *  
  */
class TLDebugModule(address: BigInt = 0)(implicit p: Parameters)
  extends TLRegisterRouter(address, "debug", Nil, beatBytes=p(XLen)/8, executable=true)(
  new TLRegBundle((), _ )    with DebugModuleBundle)(
  new TLRegModule((), _, _)  with DebugModule)


/** Synchronizers for DMI
  *  
  */


class AsyncDMIIO(implicit val p: Parameters) extends ParameterizedBundle()(p) {
  val req  = new AsyncBundle(1, new DMIReq(p(DMKey).nDMIAddrSize))
  val resp = new AsyncBundle(1, new DMIResp).flip
}

object FromAsyncDMI
{
  def apply(x: AsyncDMIIO) = {
    val out = Wire(new DMIIO()(x.p))
    out.req <> FromAsyncBundle(x.req)
    x.resp <> ToAsyncBundle(out.resp, 1)
    out
  }
}

object ToAsyncDMI
{
  def apply(x: DMIIO) = {
    val out = Wire(new AsyncDMIIO()(x.p))
    out.req <> ToAsyncBundle(x.req, 1)
    x.resp <> FromAsyncBundle(out.resp)
    out
  }
}

object AsyncDMICrossing {
  // takes from_source from the 'from' clock domain to the 'to' clock domain
  def apply(from_clock: Clock, from_reset: Bool, from_source: DMIIO, to_clock: Clock, to_reset: Bool, depth: Int = 1, sync: Int = 3) = {
    val to_sink = Wire(new DMIIO()(from_source.p))
    to_sink.req <> AsyncDecoupledCrossing(from_clock, from_reset, from_source.req, to_clock, to_reset, depth, sync)
    from_source.resp <> AsyncDecoupledCrossing(to_clock, to_reset, to_sink.resp, from_clock, from_reset, depth, sync)
    to_sink // is now to_source
  }
}

object AsyncDMIFrom { // OutsideClockDomain
  // takes from_source from the 'from' clock domain and puts it into your clock domain
  def apply(from_clock: Clock, from_reset: Bool, from_source: DMIIO, depth: Int = 1, sync: Int = 3): DMIIO = {
    val scope = AsyncScope()
    AsyncDMICrossing(from_clock, from_reset, from_source, scope.clock, scope.reset, depth, sync)
  }
}

object AsyncDMITo { // OutsideClockDomain
  // takes source from your clock domain and puts it into the 'to' clock domain
  def apply(to_clock: Clock, to_reset: Bool, source: DMIIO, depth: Int = 1, sync: Int = 3): DMIIO = {
    val scope = AsyncScope()
    AsyncDMICrossing(scope.clock, scope.reset, source, to_clock, to_reset, depth, sync)
  }
}
