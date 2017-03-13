// See LICENSE.SiFive for license details.

package uncore.ahb

import Chisel._
import chisel3.internal.sourceinfo.SourceInfo
import config._
import diplomacy._
import uncore.tilelink2._

case class AHBToTLNode() extends MixedAdapterNode(AHBImp, TLImp)(
  dFn = { case AHBMasterPortParameters(masters) =>
    TLClientPortParameters(clients = masters.map { m =>
      TLClientParameters(nodePath = m.nodePath)
    })
  },
  uFn = { mp => AHBSlavePortParameters(
    slaves = mp.managers.map { m =>
      def adjust(x: TransferSizes) = {
        if (x.contains(mp.beatBytes)) {
          TransferSizes(x.min, mp.beatBytes * AHBParameters.maxTransfer)
        } else { // larger than beatBytes requires beatBytes if misaligned
          x.intersect(TransferSizes(1, mp.beatBytes))
        }
      }

      AHBSlaveParameters(
        address       = m.address,
        resources     = m.resources,
        regionType    = m.regionType,
        executable    = m.executable,
        nodePath      = m.nodePath,
        supportsWrite = adjust(m.supportsPutFull),
        supportsRead  = adjust(m.supportsGet))},
    beatBytes = mp.beatBytes)
  })

class AHBToTL()(implicit p: Parameters) extends LazyModule
{
  val node = AHBToTLNode()

  lazy val module = new LazyModuleImp(this) {
    val io = new Bundle {
      val in = node.bundleIn
      val out = node.bundleOut
    }

    ((io.in zip io.out) zip (node.edgesIn zip node.edgesOut)) foreach { case ((in, out), (edgeIn, edgeOut)) =>
      val beatBytes = edgeOut.manager.beatBytes

      val d_send  = RegInit(Bool(false))
      val d_recv  = RegInit(Bool(false))
      val d_write = RegInit(Bool(false))
      val d_addr  = Reg(in.haddr)
      val d_size  = Reg(in.hsize)

      when (out.d.valid) { d_recv := Bool(false) }
      when (out.a.ready) { d_send := Bool(false) }

      val a_count  = RegInit(UInt(0, width = 4))
      val a_first  = a_count === UInt(0)
      val d_last   = a_first

      val burst_sizes = Seq(1, 1, 4, 4, 8, 8, 16, 16)
      val a_burst_size = Vec(burst_sizes.map(beats => UInt(log2Ceil(beats * beatBytes))))(in.hburst)
      val a_burst_mask = Vec(burst_sizes.map(beats => UInt(beats * beatBytes - 1)))(in.hburst)

      val a_burst_ok =
        in.htrans === AHBParameters.TRANS_NONSEQ && // only start burst on first AHB beat
        in.hsize  === UInt(log2Ceil(beatBytes))  && // not a narrow burst
        (in.haddr & a_burst_mask) === UInt(0)    && // address aligned to burst size
        in.hburst =/= AHBParameters.BURST_INCR   && // we know the burst length a priori
        Mux(in.hwrite,                              // target device supports the burst
          edgeOut.manager.supportsPutFullFast(in.haddr, a_burst_size),
          edgeOut.manager.supportsGetFast    (in.haddr, a_burst_size))

      val a_access = in.htrans === AHBParameters.TRANS_NONSEQ || in.htrans === AHBParameters.TRANS_SEQ
      val a_accept = in.hready && in.hsel && a_access

      when (a_accept) {
        a_count := a_count - UInt(1)
        when ( in.hwrite || a_first) { d_send := Bool(true) }
        when (!in.hwrite || a_first) { d_recv := Bool(true) }
        when (a_first) {
          a_count := Mux(a_burst_ok, a_burst_mask >> log2Ceil(beatBytes), UInt(0))
          d_write := in.hwrite
          d_addr  := in.haddr
          d_size  := Mux(a_burst_ok, a_burst_size, in.hsize)
        }
      }

      // !!! deal with invalid addresses

      out.a.valid        := d_send
      out.a.bits.opcode  := Mux(d_write, TLMessages.PutFullData, TLMessages.Get)
      out.a.bits.param   := UInt(0)
      out.a.bits.size    := d_size
      out.a.bits.source  := UInt(0)
      out.a.bits.address := d_addr
      out.a.bits.data    := in.hwdata
      out.a.bits.mask    := maskGen(d_addr, d_size, beatBytes)

      out.d.ready  := d_recv // backpressure AccessAckData arriving faster than AHB beats
      in.hrdata    := out.d.bits.data
      in.hresp     := Bool(false) // !!! need to report on last cycle only, and insert a delay?!
      in.hreadyout := Mux(d_write, (!d_send || out.a.ready) && (!d_last || !d_recv || out.d.valid), out.d.valid || !d_recv)

      // Unused channels
      out.b.ready := Bool(true)
      out.c.valid := Bool(false)
      out.e.valid := Bool(false)
    }
  }
}

object AHBToTL
{
  def apply()(x: AHBOutwardNode)(implicit p: Parameters, sourceInfo: SourceInfo): TLOutwardNode = {
    val tl = LazyModule(new AHBToTL)
    tl.node := x
    tl.node
  }
}
