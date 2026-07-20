/**
 * End-to-end check of the geometry tile pool through the REAL wasm build
 * (Phase A of the demand-geometry track): initialise the pool inside the
 * module, commit payloads from module memory, walk segments via the heap,
 * and verify bytes, refcounts, budget accounting and the header readers —
 * everything conway's InstanceAssetSource (Phase B) will sit on.
 *
 * Usage: node scripts/tile_pool_wasm_check.mjs
 * Exit 0 = all checks passed.
 */
import { createRequire } from 'node:module'

const require = createRequire( import.meta.url )

let failures = 0

function check( cond, what ) {
  if ( !cond ) {
    console.error( `FAIL: ${what}` )
    ++failures
  }
}

const loaded = require( '../Dist/ConwayGeomWasmNode.js' )
const factory = typeof loaded === 'function' ? loaded : loaded.default
const module_ = await factory()

const CHUNK = 4096
const BUDGET = 1024 * 1024 // 256 chunks

// --- init ------------------------------------------------------------------
check( !module_.geometryTilePoolInitialized(), 'pool starts uninitialised' )
check( !module_.initGeometryTilePool( 100, 8 ), 'rejects chunk below floor' )
check( module_.initGeometryTilePool( BUDGET, CHUNK ), 'init accepts budget' )
check( module_.geometryTilePoolInitialized(), 'initialised after init' )
check( module_.geometryTilePoolTotalBytes() === BUDGET, 'totalBytes = budget' )
check( module_.geometryTilePoolBytesInUse() === 0, 'starts empty' )

// --- commit raw payload from module memory ---------------------------------
// A 10 000-byte ramp: spans 3 chunks, tail partially filling the last.
const PAYLOAD = 10_000
const source = module_._malloc( PAYLOAD )
const heap = () => module_.HEAPU8

for ( let i = 0; i < PAYLOAD; ++i ) {
  heap()[ source + i ] = ( i * 7 ) & 0xFF
}

check( module_.commitGeometryTileBytes( 42, source, PAYLOAD ), 'commit fits' )
check( module_.geometryTileResident( 42 ), 'resident after commit' )
check( module_.geometryTileByteSize( 42 ) === PAYLOAD, 'logical size kept' )
check( module_.geometryTilePoolBytesInUse() === 3 * CHUNK,
    'physical use chunk-rounded' )

// Scribble over the source then free it — the tile must hold its own copy.
for ( let i = 0; i < PAYLOAD; ++i ) {
  heap()[ source + i ] = 0
}
module_._free( source )

// --- segment walk (the GPU-upload path) ------------------------------------
const segments = module_.geometryTileSegmentCount( 42 )
check( segments === 3, `3 segments (got ${segments})` )

let offset = 0
let intact = true

for ( let segment = 0; segment < segments; ++segment ) {
  const address = module_.geometryTileSegmentAddress( 42, segment )
  const length = module_.geometryTileSegmentByteLength( 42, segment )
  check( address > 0, `segment ${segment} address` )

  const view = heap().subarray( address, address + length )

  for ( let i = 0; i < length; ++i ) {
    if ( view[ i ] !== ( ( ( offset + i ) * 7 ) & 0xFF ) ) {
      intact = false
      break
    }
  }
  offset += length
}

check( offset === PAYLOAD, 'segments cover the payload exactly' )
check( intact, 'payload bytes intact across segments (own copy)' )

// --- refcount sharing ------------------------------------------------------
check( module_.retainGeometryTile( 42 ), 'retain resident tile' )
check( module_.geometryTileRefCount( 42 ) === 2, 'refcount 2' )
check( !module_.releaseGeometryTile( 42 ), 'first release keeps tile' )
check( module_.geometryTileResident( 42 ), 'still resident' )
check( module_.releaseGeometryTile( 42 ), 'last release frees' )
check( !module_.geometryTileResident( 42 ), 'gone after last release' )
check( module_.geometryTilePoolBytesInUse() === 0, 'chunks returned' )

// --- geometry-shaped payload + header readers ------------------------------
// Simulate the reified layout via raw commit: [u32 vb][u32 ib][floats][u32s].
const VERTEX_BYTES = 9 * 4 // 3 vertices x xyz floats
const INDEX_BYTES = 3 * 4  // one triangle
const total = 8 + VERTEX_BYTES + INDEX_BYTES
const geometryPayload = module_._malloc( total )

{
  const view = new DataView( module_.HEAPU8.buffer, geometryPayload, total )
  view.setUint32( 0, VERTEX_BYTES, true )
  view.setUint32( 4, INDEX_BYTES, true )

  const floats = [ 0, 0, 0, 1, 0, 0, 0, 1, 0 ]
  floats.forEach( ( v, i ) => view.setFloat32( 8 + i * 4, v, true ) );
  [ 0, 1, 2 ].forEach( ( v, i ) =>
    view.setUint32( 8 + VERTEX_BYTES + i * 4, v, true ) )
}

check( module_.commitGeometryTileBytes( 7, geometryPayload, total ),
    'geometry-shaped commit' )
module_._free( geometryPayload )

check( module_.geometryTileVertexByteLength( 7 ) === VERTEX_BYTES,
    'header vertex byte length' )
check( module_.geometryTileIndexByteLength( 7 ) === INDEX_BYTES,
    'header index byte length' )

// --- budget refusal + accounting -------------------------------------------
const big = module_._malloc( CHUNK )
let committed = 0

// Fill the pool with 1-chunk tiles until refusal.
for ( let id = 1000; id < 2000; ++id ) {
  if ( !module_.commitGeometryTileBytes( id, big, CHUNK ) ) {
    break
  }
  ++committed
}
module_._free( big )

// 256-chunk budget minus tile 7's single chunk.
check( committed === 255, `fills to budget (${committed} of 255)` )
check( module_.geometryTilePoolFreeChunks() === 0, 'no free chunks at cap' )
check( module_.geometryTilePoolFailedCommits() >= 1, 'refusal counted' )
check( module_.geometryTilePoolBytesInUse() === BUDGET, 'bytesInUse = budget' )

// Release everything; pool drains to zero (no leak, no growth).
check( module_.releaseGeometryTile( 7 ), 'release geometry tile' )
for ( let id = 1000; id < 1000 + committed; ++id ) {
  check( module_.releaseGeometryTile( id ), `release ${id}` )
}
check( module_.geometryTilePoolBytesInUse() === 0, 'drained to zero' )
check( module_.geometryTilePoolFreeChunks() === BUDGET / CHUNK, 'all chunks free' )

// --- re-init drops everything ---------------------------------------------
module_.commitGeometryTileBytes( 5, 0, 0 )
check( module_.initGeometryTilePool( BUDGET, CHUNK ), 're-init ok' )
check( !module_.geometryTileResident( 5 ), 're-init dropped tiles' )

if ( failures === 0 ) {
  console.log( 'tile_pool_wasm_check: all checks passed' )
  process.exit( 0 )
}
console.error( `tile_pool_wasm_check: ${failures} failure(s)` )
process.exit( 1 )
