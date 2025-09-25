/*
 * Decoupling:
 * https://github.com/nickcastel50/conway-geom/blob/59e9d56f6a19b5953186b78362de649437b46281/Decoupling.md
 * Ref:
 * https://github.com/IFCjs/web-ifc/blob/28681f5c4840b7ecf301e7888f98202f00adf306/src/wasm/geometry/operations/mesh_utils.h
 * */

#pragma once

#include <tinynurbs/tinynurbs.h>

#include <array>
#include <glm/glm.hpp>
#include <optional>
#include <unordered_map>
#include <vector>
#include <ranges>

#include "geometry_utils.h"
#include "tesselation_utils.h"
#include "manifold_utils.h"
#include <queue>

#define CONST_PI 3.141592653589793238462643383279502884L

namespace conway::geometry {

constexpr double MAX_DEFLECTION            = 0.000001;
constexpr double MAX_TRIANGLE_AMPLIFACTION = 32;


// TODO: review and simplify
inline void TriangulateRevolution(Geometry &geometry,
                                  std::vector<IfcBound3D> &bounds,
                                  IfcSurface &surface) {
  // First we get the revolution data

  glm::dvec3 cent = surface.RevolutionSurface.Direction[3];
  glm::dvec3 vecX = glm::normalize(surface.RevolutionSurface.Direction[0]);
  glm::dvec3 vecY = glm::normalize(surface.RevolutionSurface.Direction[1]);
  glm::dvec3 vecZ = glm::normalize(surface.RevolutionSurface.Direction[2]);

  std::vector<std::vector<glm::dvec3>> newPoints;

  double numRots = 10;

  for (int r = 0; r < numRots; r++) {
    std::vector<glm::dvec3> newList;
    newPoints.push_back(newList);
  }

  std::vector<glm::dvec3> bounding;
  std::vector<double> angleVec;
  std::vector<double> angleDsp;

  // Now we construct the bounding box of the boundary ...
  // ... by adding the middle point of all curves

  for (size_t i = 0; i < bounds.size(); i++) {
    double xx = 0;
    double yy = 0;
    double zz = 0;
    double cc = 0;
    int lastTeam = bounds[i].curve.indices[0];
    for (size_t j = 0; j < bounds[i].curve.points.size(); j++) {
      // If it is the first point of the group we close the previous group ...
      //  ... and create a new one. Else, the point is of the current group
      if (lastTeam != bounds[i].curve.indices[j] ||
          j == (bounds[i].curve.points.size() - 1)) {
        if (cc > 0) {
          xx /= cc;
          yy /= cc;
          zz /= cc;
          bounding.push_back(glm::dvec3(xx, yy, zz));
        }
        xx = bounds[i].curve.points[j].x;
        yy = bounds[i].curve.points[j].y;
        zz = bounds[i].curve.points[j].z;
        cc = 1;

        lastTeam = bounds[i].curve.indices[j];
      } else {
        xx += bounds[i].curve.points[j].x;
        yy += bounds[i].curve.points[j].y;
        zz += bounds[i].curve.points[j].z;
        cc++;
      }
    }
  }

  // There is a problem when points in the revolution are around 0 degrees
  // Numerical instabilities can make these points to jump from 0 to 360
  // It causes lots of trouble when drawing the boundaries in the revolution

  // The method presented here finds the angle of each point, measures the ...
  //  ... angular difference and then, if the difference is bigger than 180 ...
  //  ... corrects it to a lesser value. Finally it gets the first angle and ...
  //  ... adds the angular differences again, reconstructing a corrected
  //  boundary.

  // Now we find the angle of each point in the reference plane of the cylinder
  for (size_t j = 0; j < bounding.size(); j++) {
    double xx = bounding[j].x - cent.x;
    double yy = bounding[j].y - cent.y;
    double zz = bounding[j].z - cent.z;
    double dx = vecX.x * xx + vecX.y * yy + vecX.z * zz;
    double dy = vecY.x * xx + vecY.y * yy + vecY.z * zz;
    //				double dz = vecZ.x * xx + vecZ.y * yy + vecZ.z *
    // zz;
    double temp = VectorToAngle(dx, dy);
    while (temp < 0) {
      temp += 360;
    }
    while (temp > 360) {
      temp -= 360;
    }
    angleVec.push_back(temp);
  }

  for (size_t i = 0; i < angleVec.size() - 1; i++) {
    if (angleVec[i] - angleVec[i + 1] > 180) {
      angleDsp.push_back(360 - (angleVec[i] - angleVec[i + 1]));
    } else if (angleVec[i] - angleVec[i + 1] < -180) {
      angleDsp.push_back(-(angleVec[i] - angleVec[i + 1] + 360));
    } else {
      angleDsp.push_back(angleVec[i + 1] - angleVec[i]);
    }
  }

  double startDegrees = angleVec[0];
  double endDegrees = angleVec[0];

  // Add angular differences starting from the first angle. We also correct the
  // start and end angles

  double temp = angleVec[0];
  for (size_t i = 0; i < angleDsp.size(); i++) {
    temp += angleDsp[i];
    if (endDegrees < temp) {
      endDegrees = temp;
    }
    if (startDegrees > temp) {
      startDegrees = temp;
    }
  }

  // Then we use the start and end angles as bounding boxes of the boundary ...
  //  ... we will represent this bounding box.

  double startRad = startDegrees / 180 * (double)CONST_PI;
  double endRad   = endDegrees / 180 * (double)CONST_PI;
  double radSpan  = endRad - startRad;
  double radStep  = radSpan / (numRots - 1);

  for (size_t i = 0; i < surface.RevolutionSurface.Profile.curve.points.size();
       i++) {
    double xx = surface.RevolutionSurface.Profile.curve.points[i].x - cent.x;
    double yy = surface.RevolutionSurface.Profile.curve.points[i].y - cent.y;
    double zz = surface.RevolutionSurface.Profile.curve.points[i].z - cent.z;

    double dx = vecX.x * xx + vecX.y * yy + vecX.z * zz;
    double dy = vecY.x * xx + vecY.y * yy + vecY.z * zz;
    double dz = vecZ.x * xx + vecZ.y * yy + vecZ.z * zz;
    double dd = sqrt(dx * dx + dy * dy);
    for (int r = 0; r < numRots; r++) {
      double angle = startRad + r * radStep;
      double dtempX = sin(angle) * dd;
      double dtempY = cos(angle) * dd;
      double newPx = dtempX * vecX.x + dtempY * vecY.x + dz * vecZ.x + cent.x;
      double newPy = dtempX * vecX.y + dtempY * vecY.y + dz * vecZ.y + cent.y;
      double newPz = dtempX * vecX.z + dtempY * vecY.z + dz * vecZ.z + cent.z;
      glm::dvec3 newPt = glm::dvec3(newPx, newPy, newPz);
      newPoints[r].push_back(newPt);
    }
  }
  for (int r = 0; r < numRots - 1; r++) {
    int r1 = r + 1;
    for (size_t s = 0; s < newPoints[r].size() - 1; s++) {

      uint32_t a = geometry.MakeVertex( newPoints[ r ][ s ] );
      uint32_t b = geometry.MakeVertex( newPoints[ r ][ s + 1 ] );
      uint32_t c = geometry.MakeVertex( newPoints[ r1 ][ s ] );
      uint32_t d = geometry.MakeVertex( newPoints[ r1 ][ s + 1 ] );

      geometry.MakeTriangle( a, b, c );
      geometry.MakeTriangle( c, b, d );
    }
  }
}


// TODO: review and simplify
inline void TriangulateSphericalSurface(Geometry &geometry,
                                        const std::vector<IfcBound3D> &bounds,
                                        IfcSurface &surface) {
  if ( bounds.empty() ) {
    return;
  }

  double     radius = surface.SphericalSurface.Radius;
  glm::dvec3 cent   = surface.transformation[3];
  glm::dvec3 vecX   = glm::normalize(surface.transformation[0]);
  glm::dvec3 vecY   = glm::normalize(surface.transformation[1]);
  glm::dvec3 vecZ   = glm::normalize(surface.transformation[2]);

  WingedEdgeMesh< glm::dvec3 > mesh;

  tesselateDualParametrization(
    mesh,
    bounds,
    [&]( const glm::dvec3& point ) {
      // Produce a normalized vector from the centroid to the point.
      glm::dvec3 deltaCentroid = glm::normalize( point - cent );

      // we can normalize first because rotation is invariant
      // relative the centroid
      double dx = glm::dot( vecX, deltaCentroid );
      double dy = glm::dot( vecY, deltaCentroid );
      double dz = glm::dot (vecZ, deltaCentroid );

      // Project the point onto the unit sphere surface
      return glm::normalize( glm::dvec3( dx, dy, dz ) );
    },
    [&]( const glm::dvec3& normalFormVertex ) {

      const double z = ( 1 + normalFormVertex.z );

      if ( 2.0 - z < DBL_EPSILON ) {
        return glm::dvec2( 4, 4 );
      }

      return glm::normalize( glm::dvec2( normalFormVertex ) ) * z;
    },
    [&]( const glm::dvec3& normalFormVertex ) {

      const double z = ( 1 - normalFormVertex.z );

      if ( 2.0 - z < DBL_EPSILON ) {
        return glm::dvec2( 4, 4 );
      }

      return glm::normalize( glm::dvec2( normalFormVertex ) ) * z;
    },
    []( const glm::dvec3& normalFormVertex ) {

      return ( normalFormVertex.z <= 0.0 ) ? 0 : 1;
    },
    []( const glm::dvec3& normalFormVertex1,
        const glm::dvec3& normalFormVertex2,
        const glm::dvec2& paramVertex1,
        const glm::dvec2& paramVertex2 ) {

      if ( normalFormVertex1.z <= 0.0 && normalFormVertex2.z <= 0.0 ) {
        return false;
      }

      if ( normalFormVertex1.z > ( 1.0 - DBL_EPSILON ) || normalFormVertex2.z > ( 1.0 - DBL_EPSILON ) ) {
        return true;
      }

      return glm::distance( paramVertex1, paramVertex2 ) > 1.0;
    },
    []( const glm::dvec3& normalFormVertex1,
        const glm::dvec3& normalFormVertex2,
        const glm::dvec2& paramVertex1,
        const glm::dvec2& paramVertex2 ) {

        if ( normalFormVertex1.z > 0.0 && normalFormVertex2.z > 0.0 ) {
          return false;
        }

        if ( normalFormVertex1.z < ( -1.0 + DBL_EPSILON ) || normalFormVertex2.z < ( -1.0 + DBL_EPSILON ) ) {
          return true;
        }

        return glm::distance( paramVertex1, paramVertex2 ) > 1.0;
    } );

  tesselate(
    mesh,
    [&]( const glm::dvec3& point ) { 
      
      return glm::normalize( point - cent ) * radius + cent;
    },
    mesh.triangles.size() * MAX_TRIANGLE_AMPLIFACTION,
    MAX_DEFLECTION );

  appendMeshToGeometry( mesh, geometry );
}


// TODO: review and simplify
inline void TriangulateToroidalSurface(
    Geometry &geometry,
    const std::vector<IfcBound3D> &bounds,
    IfcSurface &surface) {

  if ( bounds.empty() ) {
    return;
  }

  double     majorRadius = surface.ToroidalSurface.MajorRadius;
  double     minorRadius = surface.ToroidalSurface.MinorRadius;
  glm::dvec3 cent        = surface.transformation[3];
  glm::dvec3 vecX        = glm::normalize(surface.transformation[0]);
  glm::dvec3 vecY        = glm::normalize(surface.transformation[1]);
  glm::dvec3 vecZ        = glm::normalize(surface.transformation[2]);

  WingedEdgeMesh< glm::dvec3 > mesh;

  tesselateDualParametrization(
    mesh,
    bounds,
    [&]( const glm::dvec3& point ) {

      // Produce a normalized vector from the centroid to the point.
      glm::dvec3 deltaCentroid = point - cent;

      // we can normalize first because rotation is invariant
      // relative the centroid
      double dx = glm::dot( vecX, deltaCentroid );
      double dy = glm::dot( vecY, deltaCentroid );
      double dz = glm::dot (vecZ, deltaCentroid );

      glm::dvec2 planar = glm::normalize( glm::dvec2( dx, dy ) );

      glm::dvec3 normalRingCentre = 
        glm::dvec3( 
          glm::normalize( planar ),
          0.0 );

      // Centroid on the ring.
      glm::dvec3 ringCenter =
        normalRingCentre * majorRadius;

      glm::dvec3 normalPointOnRing = glm::normalize( glm::dvec3( dx, dy, dz ) - ringCenter );

      // Project the point onto the unit sphere surface
      return ( normalRingCentre * 3.0 ) + normalPointOnRing;
    },
    [&]( const glm::dvec3& normalFormVertex ) {

      const double z = ( 1 + normalFormVertex.z );

      glm::dvec2 originalPlanar   = glm::dvec2( normalFormVertex );
      glm::dvec2 normalRingCentre = glm::normalize( originalPlanar );

      // Range [-2, 2], ring is radiug 1.5 +/- 0.5 (inner hole is 1.0, outer ring is 2)
      return normalRingCentre * ( 1.0 + z * 0.5 );
    },
    [&]( const glm::dvec3& normalFormVertex ) {

      const double z = ( 1 - normalFormVertex.z );

      glm::dvec2 originalPlanar   = glm::dvec2( normalFormVertex );
      glm::dvec2 normalRingCentre = glm::normalize( originalPlanar );

      // Range [-2, 2], ring is radiug 1.5 +/- 0.5 (inner hole is 1.0, outer ring is 2)
      return normalRingCentre * ( 1.0 + z * 0.5 );
    },
    []( const glm::dvec3& normalFormVertex ) {

      return ( normalFormVertex.z <= 0.0 ) ? 0 : 1;
    },
    []( const glm::dvec3& normalFormVertex1,
        const glm::dvec3& normalFormVertex2,
        const glm::dvec2& paramVertex1,
        const glm::dvec2& paramVertex2 ) {

      if ( normalFormVertex1.z <= 0.0 && normalFormVertex2.z <= 0.0 ) {
        return false;
      }

      return glm::distance( paramVertex1, paramVertex2 ) > 0.5;
    },
    []( const glm::dvec3& normalFormVertex1,
        const glm::dvec3& normalFormVertex2,
        const glm::dvec2& paramVertex1,
        const glm::dvec2& paramVertex2 ) {

        if ( normalFormVertex1.z >= 0.0 && normalFormVertex2.z >= 0.0 ) {
          return false;
        }

        return glm::distance( paramVertex1, paramVertex2 ) > 0.5;
    } );

  tesselate(
    mesh,
    [&]( const glm::dvec3& point ) { 
     
      // Produce a normalized vector from the centroid to the point.
      glm::dvec3 deltaCentroid = point - cent;

      // we can normalize first because rotation is invariant
      // relative the centroid
      double dx = glm::dot( vecX, deltaCentroid );
      double dy = glm::dot( vecY, deltaCentroid );
      double dz = glm::dot (vecZ, deltaCentroid );

      glm::dvec2 planar = glm::normalize( glm::dvec2( dx, dy ) );

      // Centroid on the ring.
      glm::dvec3 ringCenter =
        glm::dvec3( glm::normalize( planar ) * majorRadius, 0.0 );

      glm::dvec3 normalPointOnRing = glm::normalize( glm::dvec3( dx, dy, dz ) - ringCenter );

      glm::dvec3 pointOnIdentityRing = ringCenter + ( normalPointOnRing * minorRadius );
     
      // Move back to the original coordinate frame.
      return
        vecX * pointOnIdentityRing.x +
        vecY * pointOnIdentityRing.y +
        vecZ * pointOnIdentityRing.z + cent;
    },
    mesh.triangles.size() * MAX_TRIANGLE_AMPLIFACTION,
    MAX_DEFLECTION );

  appendMeshToGeometry( mesh, geometry );
}


// TODO: review and simplify
inline void TriangulateConicalSurface(
  Geometry &geometry,
  const std::vector<IfcBound3D> &bounds,
  IfcSurface &surface) {
  // First we get the cylinder data

  if ( bounds.empty() ) {
    return;
  }

  double radius    = surface.ConicalSurface.Radius;
  double semiAngle = surface.ConicalSurface.SemiAngle;

  double sinSemiAngle = tan( fabs( semiAngle ) );
  
  glm::dvec3 cent = surface.transformation[3];
  glm::dvec3 vecX = glm::normalize(surface.transformation[0]);
  glm::dvec3 vecY = glm::normalize(surface.transformation[1]);
  glm::dvec3 vecZ = glm::normalize(surface.transformation[2]);
  
  bool sameSense = surface.sameSense;

  if ( glm::dot( vecZ, vecX ) > 0 ) {
  
    sameSense = !sameSense;
  }

  std::vector<std::vector<glm::dvec3>> newPoints;

  double minR = DBL_MAX;
  double maxR = -DBL_MAX;

  std::priority_queue< std::pair< double, size_t > > outsideMostBoundaries;

  // Find the relative coordinates of each curve point in the cylinder reference
  // plane Only retain the max and min relative Z
  for (size_t i = 0; i < bounds.size(); i++) {

    double localMaxR = -DBL_MAX;
    double localMinR = DBL_MAX;

    for (size_t j = 0; j < bounds[i].curve.points.size(); j++) {

      glm::dvec3 pt = bounds[ i ].curve.points[ j ];
      glm::dvec3 vv = pt - cent;

      double dx = glm::dot(vecX, vv);
      double dy = glm::dot(vecY, vv);

      double dr = glm::length( glm::dvec2( dx, dy ) );
      
      localMaxR = std::max( localMaxR, dr );
      localMinR = std::min( localMinR, dr );
    }

    outsideMostBoundaries.push( std::make_pair( localMaxR, i ) );

    maxR = std::max( maxR, localMaxR );
    minR = std::min( minR, localMinR );
  }

  using Point = std::array<double, 2>;
  std::vector<std::vector<Point>> uvBoundaryValues;
  std::vector<ParameterVertex> vertices;

  WingedEdgeMesh< ParameterVertex > mesh;

  while ( !outsideMostBoundaries.empty() ) {

    std::vector<Point> points;

    size_t boundsIndex = outsideMostBoundaries.top().second;

    outsideMostBoundaries.pop();
   const IfcBound3D& bound = bounds[ boundsIndex ];

    if ( bound.curve.points.empty() ) {
      continue;
    }

    for ( const glm::dvec3& pt : bound.curve.points ) {

      glm::dvec3 vv = pt - cent;

      double dx = glm::dot(vecX, vv);
      double dy = glm::dot(vecY, vv);
      double dz = glm::dot(vecZ, vv);

      glm::dvec2 pInv = glm::dvec2( dx, dy ) / maxR;

      points.push_back({pInv.x, pInv.y});
      mesh.makeVertex( { pt, pInv } );
    }

    uvBoundaryValues.push_back( points );
  }

#if (OUTPUT_SVG_DEBUG == 1) 

    static size_t svgIndex = 0;

    size_t outputIndex = svgIndex++;

    std::ofstream svgFile( "cone_" + std::to_string( outputIndex ) + ".svg" );

    auto svgScale = []( double value ) {

      return 50 + ( 1024.0 * ( value + 1.0 ) / 2.0 ); 

    };

    svgFile << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
              << "width=\"1124\" height=\"1124\">\n";

    svgFile << "<circle cx=\"" << svgScale( 0 ) << "\" cy=\"" << svgScale( 0 )
            << "\" r=\"512\" style=\"stroke:rgb(255, 132, 0);stroke-width:2\" fill=\"none\"/>\n";

    svgFile << "<circle cx=\"" << svgScale( 0 ) << "\" cy=\"" << svgScale( 0 )
            << "\" r=\"256\" style=\"stroke:rgb(0, 0, 255);stroke-width:2\" fill=\"none\"/>\n";

    for ( const std::vector< Point >& loop  : uvBoundaryValues )
    {
      bool firstInLoop = true;
      
      glm::dvec2 lastPoint;

      for ( const Point &point : loop )
      {
        glm::dvec2 svgPoint = glm::dvec2( point[ 0 ], point[ 1 ] );

        if ( firstInLoop )
        {
          svgFile << "<polyline points=\"";
          firstInLoop = false;
        }
        else
        {
          svgFile << " ";
        }

        svgFile << svgScale( svgPoint.x ) << "," << svgScale( svgPoint.y );

        lastPoint = svgPoint;
      }

      svgFile << "\" style=\"fill:none;stroke:rgb(0,0,0);stroke-width:2\" />\n";
      
      for ( const Point &point : loop )
      {
        glm::dvec2 svgPoint = glm::dvec2( point[ 0 ], point[ 1 ] );
        
        svgFile << "<circle cx=\"" << svgScale( svgPoint.x ) << "\" cy=\"" << svgScale( svgPoint.y )
            << "\" r=\"3\" fill=\"red\"/>\n";
      }
    }

    svgFile << "</svg>\n";

    svgFile.close();

#endif

  // Triangulate projected boundary
  // Subdivide resulting triangles to increase definition
  // r indicates the level of subdivision, currently 3 you can increase it to
  // 5

  std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(uvBoundaryValues);

  for (size_t i = 0; i < indices.size(); i += 3) {

    mesh.makeTriangle( 
      indices[ i  + 0 ], 
      indices[ i  + 1 ], 
      indices[ i  + 2 ] );
  }

  tesselate(
    mesh,
    [&]( const glm::dvec3& point, [[maybe_unused]]const glm::dvec2& from ) { 
      
      glm::dvec3 vv = point - cent;
      double     dx = glm::dot( vecX, vv );
      double     dy = glm::dot( vecY, vv );    
      double     dz = glm::dot( vecZ, vv );

      glm::dvec3 coneSpacePoint = glm::dvec3( ( radius + dz * sinSemiAngle ) * glm::normalize( glm::dvec2( dx, dy ) ), dz );

      return cent + coneSpacePoint.x * vecX + coneSpacePoint.y * vecY + coneSpacePoint.z * vecZ;
    },
    mesh.triangles.size() * MAX_TRIANGLE_AMPLIFACTION,
    MAX_DEFLECTION );

  appendMeshToGeometry( mesh, geometry, sameSense );
}

// TODO: review and simplify
inline void TriangulateCylindricalSurface(Geometry &geometry,
                                          const std::vector<IfcBound3D> &bounds,
                                          IfcSurface &surface) {
  // First we get the cylinder data

  if ( bounds.empty() ) {
    return;
  }

  double radius = surface.CylinderSurface.Radius;
  glm::dvec3 cent = surface.transformation[3];
  glm::dvec3 vecX = glm::normalize(surface.transformation[0]);
  glm::dvec3 vecY = glm::normalize(surface.transformation[1]);
  glm::dvec3 vecZ = glm::normalize(surface.transformation[2]);

  bool sameSense = surface.sameSense;

  if ( glm::dot( vecZ, vecX ) > 0 ) {
  
    sameSense = !sameSense;
  }
  
  std::vector<std::vector<glm::dvec3>> newPoints;

  double minZ = DBL_MAX;
  double maxZ = -DBL_MAX;

  std::priority_queue< std::pair< double, size_t > > outsideMostBoundaries;

  if ( bounds.size() == 1 && bounds[0].curve.points.size() < 3 ) {
    // If there is no curve, we can not triangulate
    return;
  }

  // Find the relative coordinates of each curve point in the cylinder reference
  // plane Only retain the max and min relative Z
  for (size_t i = 0; i < bounds.size(); i++) {

    double localMaxZ = -DBL_MAX;
    double localMinZ = DBL_MAX;

    for (size_t j = 0; j < bounds[i].curve.points.size(); j++) {
      glm::dvec3 vv = bounds[i].curve.points[j] - cent;
      //					double dx = glm::dot(vecX, vv);
      //					double dy = glm::dot(vecY, vv);
      double dz = glm::dot(vecZ, vv);
      
      localMaxZ = std::max( localMaxZ, dz );
      localMinZ = std::min( localMinZ, dz );
    }

    outsideMostBoundaries.push( std::make_pair( localMaxZ, i ) );

    maxZ = std::max( maxZ, localMaxZ );
    minZ = std::min( minZ, localMinZ );
  }

   using Point = std::array<double, 2>;
  std::vector<std::vector<Point>> uvBoundaryValues;
  std::vector<ParameterVertex> vertices;

  WingedEdgeMesh< ParameterVertex > mesh;

  double zScale = 0.5 / ( maxZ - minZ );

  while ( !outsideMostBoundaries.empty() ) {

    std::vector<Point> points;

    size_t boundsIndex = outsideMostBoundaries.top().second;

    outsideMostBoundaries.pop();

    const IfcBound3D& bound = bounds[ boundsIndex ];

    if ( bound.curve.points.empty() ) {
      continue;
    }

    for ( const glm::dvec3& pt : bound.curve.points ) {

      glm::dvec3 vv = pt - cent;

      double dx = glm::dot(vecX, vv);
      double dy = glm::dot(vecY, vv);
      double dz = glm::dot(vecZ, vv);

      glm::dvec2 pInv =
        glm::normalize( glm::dvec2( dx, dy ) ) *
        ( 0.5 + ( dz - minZ ) * zScale );

      points.push_back({pInv.x, pInv.y});
      mesh.makeVertex( { pt, pInv } );
    } 

    uvBoundaryValues.push_back( points );
  }

#if (OUTPUT_SVG_DEBUG == 1) 

    static size_t svgIndex = 0;

    size_t outputIndex = svgIndex++;

    std::ofstream svgFile( "cylinder_" + std::to_string( outputIndex ) + ".svg" );

    auto svgScale = []( double value ) {

      return 50 + ( 1024.0 * ( value + 1.0 ) / 2.0 ); 

    };

    svgFile << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
              << "width=\"1124\" height=\"1124\">\n";

    svgFile << "<circle cx=\"" << svgScale( 0 ) << "\" cy=\"" << svgScale( 0 )
            << "\" r=\"512\" style=\"stroke:rgb(255, 132, 0);stroke-width:2\" fill=\"none\"/>\n";

    svgFile << "<circle cx=\"" << svgScale( 0 ) << "\" cy=\"" << svgScale( 0 )
            << "\" r=\"256\" style=\"stroke:rgb(0, 0, 255);stroke-width:2\" fill=\"none\"/>\n";

    for ( const std::vector< Point >& loop  : uvBoundaryValues )
    {
      bool firstInLoop = true;
      
      glm::dvec2 lastPoint;

      for ( const Point &point : loop )
      {
        glm::dvec2 svgPoint = glm::dvec2( point[ 0 ], point[ 1 ] );

        if ( firstInLoop )
        {
          svgFile << "<polyline points=\"";
          firstInLoop = false;
        }
        else
        {
          svgFile << " ";
        }

        svgFile << svgScale( svgPoint.x ) << "," << svgScale( svgPoint.y );

        lastPoint = svgPoint;
      }

      svgFile << "\" style=\"fill:none;stroke:rgb(0,0,0);stroke-width:2\" />\n";
      
      for ( const Point &point : loop )
      {
        glm::dvec2 svgPoint = glm::dvec2( point[ 0 ], point[ 1 ] );
        
        svgFile << "<circle cx=\"" << svgScale( svgPoint.x ) << "\" cy=\"" << svgScale( svgPoint.y )
            << "\" r=\"3\" fill=\"red\"/>\n";
      }
    }

    svgFile << "</svg>\n";

    svgFile.close();

#endif

  std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(uvBoundaryValues);

  for (size_t i = 0; i < indices.size(); i += 3) {

    mesh.makeTriangle( 
      indices[ i  + 0 ], 
      indices[ i  + 1 ], 
      indices[ i  + 2 ] );
  }

  tesselate(
    mesh,
    [&]( const glm::dvec3& point, [[maybe_unused]]const glm::dvec2& from ) { 
      
      glm::dvec3 vv                = point - cent;
      double     dx                = glm::dot(vecX, vv);
      double     dy                = glm::dot(vecY, vv);
      double     dz                = glm::dot(vecZ, vv);
      glm::dvec2 inPlane           = glm::dvec2( dx, dy );
      glm::dvec2 normalizedInPlane = glm::normalize( from );

      glm::dvec2 newInPlane = normalizedInPlane * radius;

      return cent + newInPlane.x * vecX + newInPlane.y * vecY + vecZ * dz;
    },
    mesh.triangles.size() * MAX_TRIANGLE_AMPLIFACTION,
    MAX_DEFLECTION );

  appendMeshToGeometry( mesh, geometry, sameSense );
}

// TODO: review and simplify
inline void TriangulateExtrusion(Geometry &geometry,
                                 std::vector<IfcBound3D> &bounds,
                                 IfcSurface &surface) {
  // NO EXAMPLE FILES ABOUT THIS CASE

  // THIS IS A SIMPLE EXTRUSION, NOT TRIMMED

  double len = surface.ExtrusionSurface.Length;
  glm::dvec3 dir = surface.ExtrusionSurface.Direction;

  if (!surface.ExtrusionSurface.Profile.isComposite) {
    for (size_t j = 0;
         j < surface.ExtrusionSurface.Profile.curve.points.size() - 1; j++) {
      int j2 = j + 1;

      double npx =
          surface.ExtrusionSurface.Profile.curve.points[j].x + dir.x * len;
      double npy =
          surface.ExtrusionSurface.Profile.curve.points[j].y + dir.y * len;
      double npz = dir.z * len;
      glm::dvec3 nptj1 = glm::dvec3(npx, npy, npz);
      npx = surface.ExtrusionSurface.Profile.curve.points[j2].x + dir.x * len;
      npy = surface.ExtrusionSurface.Profile.curve.points[j2].y + dir.y * len;
      npz = dir.z * len;
      glm::dvec3 nptj2 = glm::dvec3(npx, npy, npz);

      uint32_t nptj1i = geometry.MakeVertex( nptj1 );
      uint32_t nptj2i = geometry.MakeVertex( nptj2 );

      uint32_t a = geometry.MakeVertex(
        glm::dvec3(
          surface.ExtrusionSurface.Profile.curve.points[j].x,
          surface.ExtrusionSurface.Profile.curve.points[j].y,
          0 ) ); 

      uint32_t b = geometry.MakeVertex(
        glm::dvec3(
          surface.ExtrusionSurface.Profile.curve.points[j2].x,
          surface.ExtrusionSurface.Profile.curve.points[j2].y,
          0 ) ); 

      geometry.MakeTriangle(
        a,
        b,
        nptj1i );

      geometry.MakeTriangle(
        b,
        nptj2i,
        nptj1i );
    }
  } else {
    for (size_t i = 0; i < surface.ExtrusionSurface.Profile.profiles.size();
         i++) {
      for (size_t j = 0;
           j <
           surface.ExtrusionSurface.Profile.profiles[i].curve.points.size() - 1;
           j++) {
        int j2 = j + 1;

        double npx =
            surface.ExtrusionSurface.Profile.profiles[i].curve.points[j].x +
            dir.x * len;
        double npy =
            surface.ExtrusionSurface.Profile.profiles[i].curve.points[j].y +
            dir.y * len;
        double npz = dir.z * len;
        glm::dvec3 nptj1 = glm::dvec3(npx, npy, npz);
        npx = surface.ExtrusionSurface.Profile.profiles[i].curve.points[j2].x +
              dir.x * len;
        npy = surface.ExtrusionSurface.Profile.profiles[i].curve.points[j2].y +
              dir.y * len;
        npz = dir.z * len;
        glm::dvec3 nptj2 = glm::dvec3(npx, npy, npz);

        uint32_t nptj1i = geometry.MakeVertex( nptj1 );
        uint32_t nptj2i = geometry.MakeVertex( nptj2 );

      uint32_t a = geometry.MakeVertex(
        glm::dvec3(
                surface.ExtrusionSurface.Profile.profiles[i].curve.points[j].x,
                surface.ExtrusionSurface.Profile.profiles[i].curve.points[j].y,
                0) ); 
      
      uint32_t b = geometry.MakeVertex(
        glm::dvec3(
                surface.ExtrusionSurface.Profile.profiles[i].curve.points[j2].x,
                surface.ExtrusionSurface.Profile.profiles[i].curve.points[j2].y,
                0) ); 

        geometry.MakeTriangle(
          a,
          b,
          nptj1i);
        geometry.MakeTriangle(
          b,
          nptj2i,
          nptj1i);
      }
    }
  }
}

constexpr size_t INVERSE_GRID_SIDE   = 8.0; 
constexpr double INVERSE_GRID_SIZE_D = static_cast< double >( INVERSE_GRID_SIDE );
constexpr double INVERSE_GRID_FACTOR = 1.0 / ( INVERSE_GRID_SIZE_D - 1.0 );
constexpr double MAX_ERROR           = 0.001;
constexpr double ALPHA_ERROR         = 1e-6;
constexpr double MIN_STEP            = 1e-9;

constexpr double MAX_ITERATION       = 50;

struct RationalNurbsInverseMethod {

  glm::dvec3 grid[ INVERSE_GRID_SIDE ][ INVERSE_GRID_SIDE ];
  const tinynurbs::RationalSurface3d& surface;

  RationalNurbsInverseMethod( const tinynurbs::RationalSurface3d& srf ) : surface( srf ) {

    for ( size_t i = 0; i < INVERSE_GRID_SIDE; ++i ) {
      for ( size_t j = 0; j < INVERSE_GRID_SIDE; ++j ) {

        grid[ i ][ j ] =
          tinynurbs::surfacePoint(
            srf,
            static_cast< double >( i ) * INVERSE_GRID_FACTOR,
            static_cast< double > ( j ) * INVERSE_GRID_FACTOR );
      }
    }
  }

  glm::dvec2 operator()( const glm::dvec3& point ) {

    glm::dvec2 bestGuess;
    glm::dvec3 bestPoint;
    double     minDistance2 = DBL_MAX;
    
    // Take initial guess from the grid.
    for ( size_t i = 0; i < INVERSE_GRID_SIDE; ++i ) {
      for ( size_t j = 0; j < INVERSE_GRID_SIDE; ++j ) {

        glm::dvec3 deltaP = grid[ i ][ j ] - point;

        double distance2 = glm::dot( deltaP, deltaP );

        if ( distance2 < minDistance2 ) {

          bestGuess =
            glm::dvec2(
              static_cast< double >( i ) * INVERSE_GRID_FACTOR,
              static_cast< double >( j ) * INVERSE_GRID_FACTOR );

          bestPoint    = grid[ i ][ j ];
          minDistance2 = distance2;
        }
      }
    }

    size_t iteration = 0;

    double damping = 1e-6;

    while ( minDistance2 > MAX_ERROR * MAX_ERROR && iteration++ < MAX_ITERATION ) {

      glm::dvec3 deltaP = bestPoint - point;

      if ( minDistance2 <= MAX_ERROR * MAX_ERROR ) {
        break;
      }

      auto [dU, dV] = tinynurbs::surfaceTangent( surface, bestGuess.x, bestGuess.y );

      glm::dmat2x3 jacobianT( dU, dV ); // Jacobian
      glm::dmat3x2 jacobian = glm::transpose( jacobianT );   // Transposed Jacobian

      glm::dmat2x2 jtj = jacobian * jacobianT; // J^T * J
      glm::dvec2   jte = jacobian * deltaP;    // J^T * e

      jtj[ 0 ][ 0 ] += damping;
      jtj[ 1 ][ 1 ] += damping;

      glm::dvec2 deltaUV = robust_2x2_solve( jtj, jte );

      double alpha = 1.0;
      double phi   = 0.5 * minDistance2;

      bool success = false;

      while ( alpha > ALPHA_ERROR ) {

        glm::dvec2 newGuessUV = bestGuess - deltaUV * alpha;

        newGuessUV.x = std::clamp( newGuessUV.x, 0.0, 1.0 );
        newGuessUV.y = std::clamp( newGuessUV.y, 0.0, 1.0 );

        glm::dvec3 newPoint =
          tinynurbs::surfacePoint( surface, newGuessUV.x, newGuessUV.y );

        glm::dvec3 newDeltaP = newPoint - point;

        double newDistance2 = glm::dot( newDeltaP, newDeltaP );

        if ( newDistance2 < minDistance2 - MAX_ERROR * alpha * glm::dot( deltaUV, jte ) ) {

          bestPoint = newPoint;
          bestGuess = newGuessUV;
          damping  *= 0.1;
          
          minDistance2 = std::min( newDistance2, newDistance2 );
          success = true;
          break;
        }

        alpha *= 0.5;
      }

      if ( !success ) {
        damping *= 10.0;
        continue;
      }

      if ( glm::dot( deltaUV, deltaUV ) < MIN_STEP * MIN_STEP ) {
        break;
      }
    }

    return bestGuess;
  }

};


/*inline double InverseMethod(glm::dvec3 pt, const tinynurbs::RationalSurface3d& srf,
                            double pr, double rotations, double minError,
                            double maxError, double &fU, double &fV,
                            double &divisor, double maxDistance) {
  while (maxDistance > maxError && divisor < 10000) {
    for (double r = 1; r < 5; r++) {
      int round = 0;
      while (maxDistance > minError && round < 3) {
      //  printf("maxError: %.3f\n", maxError);
       // printf("minError: %.3f\n", minError);
      //  printf("round: %i\n", round);
        for (double i = 0; i < rotations; i++) {
          double rads = (i / rotations) * (double)CONST_PI * 2;
          double incU = glm::sin(rads) / (r * r * divisor);
          double incV = glm::cos(rads) / (r * r * divisor);
          if (pr > 1) {
            incV *= pr;
          } else {
            incU /= pr;
          }
          bool repeat = true;
          while (repeat) {
            double ffU = fU + incU;
            double ffV = fV + incV;
            glm::highp_dvec3 pt00 = tinynurbs::surfacePoint(srf, ffU, ffV);
            double di = glm::distance(pt00, pt);
            if (di < maxDistance) {
              maxDistance = di;
              fU = ffU;
              fV = ffV;
            } else {
              repeat = false;
            }
          }
        }
        round++;
      }
    }
    divisor *= 3;
    // printf("divisor: %.3f\n", divisor);
    // printf("maxError: %.3f\n", maxError);
    // printf("minError: %.3f\n", minError);
  }
  return maxDistance;
}*/

// inline glm::dvec2 BSplineInverseEvaluation(glm::dvec3 pt,
//                                            const tinynurbs::RationalSurface3d& srf,
//                                            double scaling) {
//   glm::highp_dvec3 ptc = tinynurbs::surfacePoint(srf, 0.0, 0.0);
//   glm::highp_dvec3 pth = tinynurbs::surfacePoint(srf, 1.0, 0.0);
//   glm::highp_dvec3 ptv = tinynurbs::surfacePoint(srf, 0.0, 1.0);

//   double dh = glm::distance(ptc, pth);
//   double dv = glm::distance(ptc, ptv);
//   double pr = (dh + 1) / (dv + 1);

//   double minError = 0.00001;
//   double maxError = 0.001;
//   double rotations = 6;

//   double fU = 0.5;
//   double fV = 0.5;
//   double divisor = 100.0;
//   double maxDistance = 1e+100;

//   //printf("scaling: %.3f\n", scaling);
//   maxDistance =
//     InverseMethod(
//         pt,
//         srf,
//         pr,
//         rotations,
//         minError / scaling,
//         maxError / scaling,
//         fU,
//         fV,
//         divisor,
//         maxDistance );

//   return glm::dvec2(fU, fV);
// }

// TODO: review and simplify
inline void TriangulateBspline(Geometry &geometry,
                               const std::vector<IfcBound3D> &bounds,
                               IfcSurface &surface, double scaling) {

//  printf( "Triangulating BSpline Surface\n" );

  tinynurbs::RationalSurface3d srf;
  srf.degree_u = surface.BSplineSurface.UDegree;
  srf.degree_v = surface.BSplineSurface.VDegree;
  size_t num_u = surface.BSplineSurface.ControlPoints.size();
  size_t num_v = surface.BSplineSurface.ControlPoints[0].size();

  std::vector<glm::dvec3> controlPoints;
  for (std::vector<glm::dvec3> row : surface.BSplineSurface.ControlPoints) {
    for (glm::dvec3 point : row) {
      controlPoints.push_back({point.x, point.y, point.z});
    }
  }
  srf.control_points = tinynurbs::array2(num_u, num_v, controlPoints);

  std::vector<double> weights;
  // for (std::vector<double> row : surface.BSplineSurface.WeightPoints) {
  for (std::vector<double> row : surface.BSplineSurface.Weights) {
    for (double weight : row) {
      weights.push_back(weight);
    }
  }

  if (weights.size() != num_u * num_v) {
    for (size_t i = 0; i < num_u * num_v; i++) {
      weights.push_back(1.0);
    }
  }
  
  srf.weights = tinynurbs::array2(num_u, num_v, weights);

  for (size_t i = 0; i < surface.BSplineSurface.UMultiplicity.size(); i++) {
    for (size_t r = 0; r < surface.BSplineSurface.UMultiplicity[i]; r++) {
      srf.knots_u.push_back(surface.BSplineSurface.UKnots[i]);
    }
  }

  for (size_t i = 0; i < surface.BSplineSurface.VMultiplicity.size(); i++) {
    for (size_t r = 0; r < surface.BSplineSurface.VMultiplicity[i]; r++) {
      srf.knots_v.push_back(surface.BSplineSurface.VKnots[i]);
    }
  }

  // If the NURBS surface is valid we continue
  

//  printf( "Evaluating inverse parameter space\n" );

  RationalNurbsInverseMethod bSplineInverseEvaluation( srf );

  if (tinynurbs::surfaceIsValid(srf)) {
    // Find projected boundary using NURBS inverse evaluation

    using Point = std::array<double, 2>;
    std::vector<std::vector<Point>> uvBoundaryValues;

    std::vector<ParameterVertex> vertices;

    WingedEdgeMesh< ParameterVertex > mesh;

    for ( size_t i = 0; i < bounds.size(); ++i ) {

      std::vector<Point> points;

      for (size_t j = 0; j < bounds[i].curve.points.size(); j++) {
        glm::dvec3 pt = bounds[i].curve.points[j];

        //hack 
        pt.x *= scaling;
        pt.y *= scaling;
        pt.z *= scaling;

        glm::dvec2 pInv = bSplineInverseEvaluation( pt );

        points.push_back({pInv.x, pInv.y});
        mesh.makeVertex( { pt, pInv } );
      }

      uvBoundaryValues.push_back(points);
    }

  //  printf( "Earcutting parameter space %zu\n", mesh.vertices.size() );

    // Triangulate projected boundary
    // Subdivide resulting triangles to increase definition
    // r indicates the level of subdivision, currently 3 you can increase it to
    // 5

    std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(uvBoundaryValues);

    for ( size_t i = 0; i < indices.size(); i += 3 ) {

      mesh.makeTriangle( 
        indices[ i  + 0 ], 
        indices[ i  + 1 ], 
        indices[ i  + 2 ] );
    }
    
  //  printf( "Tesselating BSpline Surface\n" );

    tesselate(
      mesh,
      [&srf]( [[maybe_unused]]const glm::dvec3&, const glm::dvec2& from ) { 
        return tinynurbs::surfacePoint(srf, from.x, from.y); 
      },
      mesh.triangles.size() * MAX_TRIANGLE_AMPLIFACTION,
      MAX_DEFLECTION );

    appendMeshToGeometry( mesh, geometry, !surface.sameSense );

  //  printf( "Tesselated BSpline Surface with %zu triangles\n", mesh.triangles.size() );


  } else {
    Logger::logError( "Surface was not valid!\n");
  }
}
}  // namespace conway::geometry
