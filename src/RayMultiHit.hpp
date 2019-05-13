#ifndef ADMM_RAYMULTIHIT_HPP
#define ADMM_RAYMULTIHIT_HPP

#include "MCL/Visitor.hpp"

namespace mcl
{
    namespace bvh
    {

// Raycast with multi-hit (counter)
template <typename T>
class RayMultiHit : public Visitor<T,3> {
typedef Eigen::AlignedBox<T,3> AABB;
public:
	raycast::Ray<T> ray;
	int skip_vert_idx; // vert index to skip (for self collision)
	int hit_count; // number of intersections
	const T *verts;
	const int *inds;
	RayMultiHit( Vec3<T> point_, const T *verts_, const int *inds_ );
	bool hit_aabb( const AABB &aabb );
	bool hit_prim( int prim );
	bool check_left_first( const AABB &left, const AABB &right );
};

template <typename T> 
RayMultiHit<T>::RayMultiHit( Vec3<T> point_, const T *verts_, const int *inds_ ) :
	ray(point_, Vec3<T>(0,1,0)), verts(verts_), inds(inds_), skip_vert_idx(-1), hit_count(0) {}

template <typename T> 
bool RayMultiHit<T>::hit_aabb( const AABB &aabb ){
	return raycast::ray_aabb( &ray, aabb.min(), aabb.max() );
}

template <typename T> 
bool RayMultiHit<T>::hit_prim( int prim ){

	Vec3i tri( inds[prim*3+0], inds[prim*3+1], inds[prim*3+2] );
	for( int i=0; i<3; ++i ){ if( tri[i]==skip_vert_idx ){ return false; } }
	Vec3<T> v0( verts[tri[0]*3+0], verts[tri[0]*3+1], verts[tri[0]*3+2] );
	Vec3<T> v1( verts[tri[1]*3+0], verts[tri[1]*3+1], verts[tri[1]*3+2] );
	Vec3<T> v2( verts[tri[2]*3+0], verts[tri[2]*3+1], verts[tri[2]*3+2] );

	// We'll use a fresh payload each time so we don't only find
	// nearest intersections. This lets us count how many times we
	// hit a surface, and sort them from nearest to furthest.
	raycast::Payload<T> tmpPayload;
	bool hit = raycast::ray_triangle( &ray, v0, v1, v2, &tmpPayload );
	if( hit ){ hit_count++; }
	return false; // return false to keep checking other tris
}

template <typename T> 
bool RayMultiHit<T>::check_left_first( const AABB &left, const AABB &right ){
	return raycast::ray_aabb( &ray, left.min(), left.max() );
}
}
}

#endif
