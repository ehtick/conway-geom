import { Deletable } from '../deletable'
import { NativeTransform } from '../native_transform'
import { Vector3 } from '../vector3'
import { BSplineSurface } from './bspline_surface'
import { ConicalSurface } from './conical_surface'
import { CylinderSurface } from './cylinder_surface'
import { ExtrusionSurface } from './extrusion_surface'
import { RevolutionSurface } from './revolution_surface'
import { SphericalSurface } from './spherical_surface'
import { ToroidalSurface } from './toroidal_surface'


/**
 * Surface object parameter set representing all possible surface
 * definitions of a surface for b-rep.
 */
export interface SurfaceObject extends Deletable {

  transformation: NativeTransform
  bspline: BSplineSurface
  cylinder: CylinderSurface
  cone: ConicalSurface
  sphere: SphericalSurface
  torus: ToroidalSurface
  revolution: RevolutionSurface
  extrusion: ExtrusionSurface
  sameSense: boolean

  normal(): Vector3
}
