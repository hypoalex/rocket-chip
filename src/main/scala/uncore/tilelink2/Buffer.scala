// See LICENSE.SiFive for license details.

package uncore.tilelink2

import Chisel._
import chisel3.internal.sourceinfo.SourceInfo
import config._
import diplomacy._
import scala.math.{min,max}

case class TLBufferConfig(depth: Int, flow: Boolean, pipe: Boolean)
{
  require (depth >= 0)
  def isDefined = depth > 0
  def latency = if (isDefined && !flow) 1 else 0
}

object TLBufferConfig
{
  implicit def apply(depth: Int): TLBufferConfig = TLBufferConfig(depth, false, false)

  val default = TLBufferConfig(2)
  val none    = TLBufferConfig(0)
  val flow    = TLBufferConfig(1, true, false)
  val pipe    = TLBufferConfig(1, false, true)
}

class TLBuffer(
  a: TLBufferConfig,
  b: TLBufferConfig,
  c: TLBufferConfig,
  d: TLBufferConfig,
  e: TLBufferConfig)(implicit p: Parameters) extends LazyModule
{
  def this(ace: TLBufferConfig, bd: TLBufferConfig)(implicit p: Parameters) = this(ace, bd, ace, bd, ace)
  def this(abcde: TLBufferConfig)(implicit p: Parameters) = this(abcde, abcde)
  def this()(implicit p: Parameters) = this(TLBufferConfig.default)

  val node = TLAdapterNode(
    clientFn  = { p => p.copy(minLatency = p.minLatency + b.latency + c.latency) },
    managerFn = { p => p.copy(minLatency = p.minLatency + a.latency + d.latency) })

  lazy val module = new LazyModuleImp(this) {
    val io = new Bundle {
      val in  = node.bundleIn
      val out = node.bundleOut
    }

    def buffer[T <: Data](config: TLBufferConfig, data: DecoupledIO[T]): DecoupledIO[T] = {
      if (config.isDefined) {
        Queue(data, config.depth, pipe=config.pipe, flow=config.flow)
      } else {
        data
      }
    }

    ((io.in zip io.out) zip (node.edgesIn zip node.edgesOut)) foreach { case ((in, out), (edgeIn, edgeOut)) =>
      out.a <> buffer(a, in .a)
      in .d <> buffer(d, out.d)

      if (edgeOut.manager.anySupportAcquireB && edgeOut.client.anySupportProbe) {
        in .b <> buffer(b, out.b)
        out.c <> buffer(c, in .c)
        out.e <> buffer(e, in .e)
      } else {
        in.b.valid := Bool(false)
        in.c.ready := Bool(true)
        in.e.ready := Bool(true)
        out.b.ready := Bool(true)
        out.c.valid := Bool(false)
        out.e.valid := Bool(false)
      }
    }
  }
}

object TLBuffer
{
  // applied to the TL source node; y.node := TLBuffer(x.node)
  def apply()                                       (x: TLOutwardNode)(implicit p: Parameters, sourceInfo: SourceInfo): TLOutwardNode = apply(TLBufferConfig.default)(x)
  def apply(abcde: TLBufferConfig)                  (x: TLOutwardNode)(implicit p: Parameters, sourceInfo: SourceInfo): TLOutwardNode = apply(abcde, abcde)(x)
  def apply(ace: TLBufferConfig, bd: TLBufferConfig)(x: TLOutwardNode)(implicit p: Parameters, sourceInfo: SourceInfo): TLOutwardNode = apply(ace, bd, ace, bd, ace)(x)
  def apply(
      a: TLBufferConfig,
      b: TLBufferConfig,
      c: TLBufferConfig,
      d: TLBufferConfig,
      e: TLBufferConfig)(x: TLOutwardNode)(implicit p: Parameters, sourceInfo: SourceInfo): TLOutwardNode = {
    val buffer = LazyModule(new TLBuffer(a, b, c, d, e))
    buffer.node := x
    buffer.node
  }
}
