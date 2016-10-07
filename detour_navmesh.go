package detour

import (
	"bytes"

	"github.com/go-gl/mathgl/mgl32"
)

// A handle to a polygon within a navigation mesh tile
type PolyRef uint

// A handle to a tile within a navigation mesh
type TileRef uint

// The maximum number of vertices per navigation polygon
const DT_VERTS_PER_POLYGON = 6

const (
	// A magic number used to detect compatibility of navigation tile data.
	DT_NAVMESH_MAGIC = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V'

	// A version number used to detect compatibility of navigation tile data.
	DT_NAVMESH_VERSION = 7

	// A magic number used to detect the compatibility of navigation tile states.
	DT_NAVMESH_STATE_MAGIC = 'D'<<24 | 'N'<<16 | 'M'<<8 | 'S'

	// A version number used to detect compatibility of navigation tile states.
	DT_NAVMESH_STATE_VERSION = 1
)

const (
	// A flag that indicates that an entity links to an external entity.
	// (E.g. A polygon edge is a portal that links to another polygon.)
	DT_EXT_LINK = 0x8000

	// A value that indicates the entity does not link to anything.
	DT_NULL_LINK = 0xffffffff

	// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
	DT_OFFMESH_CON_BIDIR = 1

	// The maximum number of user defined area ids.
	// @ingroup detour
	DT_MAX_AREAS = 64
)

// Tile flags used for various functions and fields.
// For an example, see dtNavMesh::addTile().

type TileFlags int

const (
	// The navigation mesh owns the tile memory and is responsible for freeing it.
	DT_TILE_FREE_DATA TileFlags = 0x01
)

// Vertex flags returned by dtNavMeshQuery::findStraightPath.
type StraightPathFlags int

const (
	DT_STRAIGHTPATH_START              StraightPathFlags = 0x01 //< The vertex is the start position in the path.
	DT_STRAIGHTPATH_END                                  = 0x02 //< The vertex is the end position in the path.
	DT_STRAIGHTPATH_OFFMESH_CONNECTION                   = 0x04 //< The vertex is the start of an off-mesh connection.
)

// Options for dtNavMeshQuery::findStraightPath.
type StraightPathOptions int

const (
	DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01 //< Add a vertex at every polygon edge crossing where area changes.
	DT_STRAIGHTPATH_ALL_CROSSINGS  = 0x02 //< Add a vertex at every polygon edge crossing.
)

// Options for dtNavMeshQuery::initSlicedFindPath and updateSlicedFindPath
type FindPathOptions int

const (
	DT_FINDPATH_ANY_ANGLE = 0x02 //< use raycasts during pathfind to "shortcut" (raycast still consider costs)
)

// Options for dtNavMeshQuery::raycast
type RaycastOptions int

const (
	DT_RAYCAST_USE_COSTS = 0x01 //< Raycast should calculate movement cost along the ray and fill RaycastHit::cost
)

// Limit raycasting during any angle pahfinding
// The limit is given as a multiple of the character radius
const DT_RAY_CAST_LIMIT_PROPORTIONS = 50.0

// Flags representing the type of a navigation mesh polygon.
type PolyTypes int

const (
	// The polygon is a standard convex polygon that is part of the surface of the mesh.
	DT_POLYTYPE_GROUND = 0
	// The polygon is an off-mesh connection consisting of two vertices.
	DT_POLYTYPE_OFFMESH_CONNECTION = 1
)

// Defines a polygon within a dtMeshTile object.
// @ingroup detour
type Poly struct {
	// Index to first link in linked list. (Or #DT_NULL_LINK if there is no link.)
	FirstLink uint

	// The indices of the polygon's vertices.
	// The actual vertices are located in dtMeshTile::verts.
	Verts [DT_VERTS_PER_POLYGON]uint16

	// Packed data representing neighbor polygons references and flags for each edge.
	Neis [DT_VERTS_PER_POLYGON]uint16

	// The user defined polygon flags.
	Flags uint16

	// The number of vertices in the polygon.
	VertCount uint8

	// The bit packed area id and polygon type.
	// @note Use the structure's set and get methods to acess this value.
	areaAndtype uint8
}

// Sets the user defined area id. [Limit: < #DT_MAX_AREAS]
func (p *Poly) SetArea(a uint8) {
	p.areaAndtype = (p.areaAndtype & 0xc0) | (a & 0x3f)
}

// Sets the polygon type. (See: #dtPolyTypes.)
func (p *Poly) SetType(t uint8) {
	p.areaAndtype = (p.areaAndtype & 0x3f) | (t << 6)
}

// Gets the user defined area id.
func (p *Poly) GetArea() uint8 {
	return p.areaAndtype & 0x3f
}

// Gets the polygon type. (See: #dtPolyTypes)
func (p *Poly) GetType() uint8 {
	return p.areaAndtype >> 6
}

// Defines the location of detail sub-mesh data within a dtMeshTile.
type PolyDetail struct {
	VertBase  uint  //< The offset of the vertices in the dtMeshTile::detailVerts array.
	TriBase   uint  //< The offset of the triangles in the dtMeshTile::detailTris array.
	VertCount uint8 //< The number of vertices in the sub-mesh.
	TriCount  uint8 //< The number of triangles in the sub-mesh.
}

// Defines a link between polygons.
// @note This structure is rarely if ever used by the end user.
// @see dtMeshTile
type Link struct {
	ref  PolyRef //< Neighbour reference. (The neighbor that is linked to.)
	next uint    //< Index of the next link.
	edge uint8   //< Index of the polygon edge that owns this link.
	side uint8   //< If a boundary link, defines on which side the link is.
	bmin uint8   //< If a boundary link, defines the minimum sub-edge area.
	bmax uint8   //< If a boundary link, defines the maximum sub-edge area.
}

// Bounding volume node.
// @note This structure is rarely if ever used by the end user.
// @see dtMeshTile
type BVNode struct {
	bmin [3]uint16 //< Minimum bounds of the node's AABB. [(x, y, z)]
	bmax [3]uint16 //< Maximum bounds of the node's AABB. [(x, y, z)]
	i    int       //< The node's index. (Negative for escape sequence.)
}

// Defines an navigation mesh off-mesh connection within a dtMeshTile object.
// An off-mesh connection is a user defined traversable connection made up to two vertices.
type OffMeshConnection struct {
	// The endpoints of the connection. [(ax, ay, az, bx, by, bz)]
	Pos [6]float32

	// The radius of the endpoints. [Limit: >= 0]
	Rad float32

	// The polygon reference of the connection within the tile.
	Poly uint16

	// Link flags.
	// @note These are not the connection's user defined flags. Those are assigned via the
	// connection's dtPoly definition. These are link flags used for internal purposes.
	Flags uint8

	// End point side.
	Side uint8

	// The id of the offmesh connection. (User assigned when the navigation mesh is built.)
	UserId uint
}

// Provides high level information related to a dtMeshTile object.
type MeshHeader struct {
	Magic           int  //< Tile magic number. (Used to identify the data format.)
	Version         int  //< Tile data format version number.
	X               int  //< The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	Y               int  //< The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	Layer           int  //< The layer of the tile within the dtNavMesh tile grid. (x, y, layer)
	UserId          uint //< The user defined id of the tile.
	PolyCount       int  //< The number of polygons in the tile.
	VertCount       int  //< The number of vertices in the tile.
	MaxLinkCount    int  //< The number of allocated links.
	DetailMeshCount int  //< The number of sub-meshes in the detail mesh.

	// The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
	DetailVertCount int

	DetailTriCount  int        //< The number of triangles in the detail mesh.
	BVNodeCount     int        //< The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
	OffMeshConCount int        //< The number of off-mesh connections.
	OffMeshBase     int        //< The index of the first polygon which is an off-mesh connection.
	WalkableHeight  float32    //< The height of the agents using the tile.
	WalkableRadius  float32    //< The radius of the agents using the tile.
	WalkableClimb   float32    //< The maximum climb height of the agents using the tile.
	BMin            mgl32.Vec3 //< The minimum bounds of the tile's AABB. [(x, y, z)]
	BMax            mgl32.Vec3 //< The maximum bounds of the tile's AABB. [(x, y, z)]

	// The bounding volume quantization factor.
	BVQuantFactor float32
}

func (m *MeshHeader) Unmarshal(data []byte) (err error) {
	r := UtilReader{r: bytes.NewBuffer(data)}

	if m.Magic, err = r.ReadInt(); err != nil {
		return err
	}
	if m.Version, err = r.ReadInt(); err != nil {
		return err
	}
	if m.X, err = r.ReadInt(); err != nil {
		return err
	}
	if m.Y, err = r.ReadInt(); err != nil {
		return err
	}
	if m.Layer, err = r.ReadInt(); err != nil {
		return err
	}
	if m.UserId, err = r.ReadUint(); err != nil {
		return err
	}
	if m.PolyCount, err = r.ReadInt(); err != nil {
		return err
	}
	if m.VertCount, err = r.ReadInt(); err != nil {
		return err
	}
	if m.MaxLinkCount, err = r.ReadInt(); err != nil {
		return err
	}
	if m.DetailMeshCount, err = r.ReadInt(); err != nil {
		return err
	}
	if m.DetailVertCount, err = r.ReadInt(); err != nil {
		return err
	}
	if m.DetailTriCount, err = r.ReadInt(); err != nil {
		return err
	}
	if m.BVNodeCount, err = r.ReadInt(); err != nil {
		return err
	}
	if m.OffMeshConCount, err = r.ReadInt(); err != nil {
		return err
	}
	if m.OffMeshBase, err = r.ReadInt(); err != nil {
		return err
	}
	if m.WalkableHeight, err = r.ReadFloat(); err != nil {
		return err
	}
	if m.WalkableRadius, err = r.ReadFloat(); err != nil {
		return err
	}
	if m.WalkableClimb, err = r.ReadFloat(); err != nil {
		return err
	}
	for i := range m.BMin {
		if m.BMin[i], err = r.ReadFloat(); err != nil {
			return err
		}
	}
	for i := range m.BMin {
		if m.BMax[i], err = r.ReadFloat(); err != nil {
			return err
		}
	}
	if m.BVQuantFactor, err = r.ReadFloat(); err != nil {
		return err
	}
	return nil
}

// Defines a navigation mesh tile.
type MeshTile struct {
	Salt uint //< Counter describing modifications to the tile.

	LinksFreeList uint         //< Index to the next free link.
	Header        *MeshHeader  //< The tile header.
	Polys         []Poly       //< The tile polygons. [Size: dtMeshHeader::polyCount]
	Verts         []float32    //< The tile vertices. [Size: dtMeshHeader::vertCount]
	Links         []Link       //< The tile links. [Size: dtMeshHeader::maxLinkCount]
	DetailMeshes  []PolyDetail //< The tile's detail sub-meshes. [Size: dtMeshHeader::detailMeshCount]

	// The detail mesh's unique vertices. [(x, y, z) * dtMeshHeader::detailVertCount]
	DetailVerts []float32

	// The detail mesh's triangles. [(vertA, vertB, vertC) * dtMeshHeader::detailTriCount]
	DetailTris []uint8

	// The tile bounding volume nodes. [Size: dtMeshHeader::bvNodeCount]
	// (Will be null if bounding volumes are disabled.)
	BVTree []BVNode

	OffMeshCons []OffMeshConnection //< The tile off-mesh connections. [Size: dtMeshHeader::offMeshConCount]

	Data []byte //< The tile data. (Not directly accessed under normal situations.)
	//DataSize int       //< Size of the tile data.
	Flags int       //< Tile flags. (See: #dtTileFlags)
	Next  *MeshTile //< The next free tile, or the next tile in the spatial grid.
}

// Configuration parameters used to define multi-tile navigation meshes.
// The values are used to allocate space during the initialization of a navigation mesh.
// @see dtNavMesh::init()
type NavMeshParams struct {
	Orig       mgl32.Vec3 //< The world space origin of the navigation mesh's tile space. [(x, y, z)]
	TileWidth  float32    //< The width of each tile. (Along the x-axis.)
	TileHeight float32    //< The height of each tile. (Along the z-axis.)
	MaxTiles   uint       //< The maximum number of tiles the navigation mesh can contain.
	MaxPolys   uint       //< The maximum number of polygons each tile can contain.
}

// A navigation mesh based on tiles of convex polygons.
type NavMesh struct {
	params                NavMeshParams //< Current initialization params. TODO: do not store this info twice.
	orig                  mgl32.Vec3    //< Origin of the tile (0,0)
	tileWidth, tileHeight float32       //< Dimensions of each tile.
	maxTiles              int           //< Max number of tiles.
	tileLutSize           int           //< Tile hash lookup size (must be pot).
	tileLutMask           int           //< Tile hash lookup mask.

	posLookup []*MeshTile //< Tile hash lookup.
	nextFree  *MeshTile   //< Freelist of tiles.
	tiles     []MeshTile  //< List of tiles.

	//#ifndef DT_POLYREF64
	saltBits uint //< Number of salt bits in the tile ID.
	tileBits uint //< Number of tile bits in the tile ID.
	polyBits uint //< Number of poly bits in the tile ID.
	//#endif

	//dtNavMesh();
	//~dtNavMesh();
}

// @{
// @name Initialization and Tile Management

// Initializes the navigation mesh for tiled use.
//  @param[in]	params		Initialization parameters.
// @return The status flags for the operation.
func (n *NavMesh) InitWithParams(params NavMeshParams) error {
	n.params = params
	copy(n.orig[:], params.Orig[:])
	n.tileWidth, n.tileHeight = params.TileWidth, params.TileHeight

	// Init tiles
	n.maxTiles = int(params.MaxTiles)
	n.tileLutSize = int(nextPow2(params.MaxTiles / 4))
	if n.tileLutSize <= 0 {
		n.tileLutSize = 1
	}
	n.tileLutMask = n.tileLutSize - 1

	n.tiles = make([]MeshTile, n.maxTiles)
	n.posLookup = make([]*MeshTile, n.tileLutSize)
	n.nextFree = nil

	for i := n.maxTiles - 1; i >= 0; i-- {
		n.tiles[i].Salt = 1
		n.tiles[i].Next = n.nextFree
		n.nextFree = &n.tiles[i]
	}

	// Init ID generator values.
	//#ifndef DT_POLYREF64
	n.tileBits = ilog2(nextPow2(uint(params.MaxTiles)))
	n.polyBits = ilog2(nextPow2(uint(params.MaxPolys)))

	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	n.saltBits = min(31, 32-n.tileBits-n.polyBits)

	if n.saltBits < 10 {
		return statusError(DT_FAILURE | DT_INVALID_PARAM)
	}
	//#endif

	return nil
}

// Initializes the navigation mesh for single tile use.
//  @param[in]	data		Data of the new tile. (See: #dtCreateNavMeshData)
//  @param[in]	flags		The tile flags. (See: #dtTileFlags)
// @return The status flags for the operation.
//  @see dtCreateNavMeshData
func (n *NavMesh) Init(data []byte, flags int) error {
	// Make sure the data is in right format.
	var header MeshHeader
	if err := header.Unmarshal(data); err != nil {
		return err
	}

	if header.Magic != DT_NAVMESH_MAGIC {
		return statusError(DT_FAILURE | DT_WRONG_MAGIC)
	} else if header.Version != DT_NAVMESH_VERSION {
		return statusError(DT_FAILURE | DT_WRONG_VERSION)
	}

	var params NavMeshParams
	copy(params.Orig[:], header.BMin[:])
	params.TileWidth = header.BMax[0] - header.BMin[0]
	params.TileHeight = header.BMax[2] - header.BMin[2]
	params.MaxTiles = 1
	params.MaxPolys = header.PolyCount

	if err := n.InitWithParams(params); err != nil {
		return err
	}

	return n.AddTile(data, flags, 0)
}

// The navigation mesh initialization params.
func (n *NavMesh) GetParams() NavMeshParams {
	return n.params
}

// Adds a tile to the navigation mesh.
//  @param[in]		data		Data for the new tile mesh. (See: #dtCreateNavMeshData)
//  @param[in]		flags		Tile flags. (See: #dtTileFlags)
//  @param[in]		lastRef		The desired reference for the tile. (When reloading a tile.) [opt] [Default: 0]
//  @param[out]	result		The tile reference. (If the tile was succesfully added.) [opt]
// @return The status flags for the operation.
func (n *NavMesh) AddTile(data []byte, flags int, lastRef TileRef) (TileRef, error) {
	// Make sure the data is in right format.
	var header MeshHeader
	if err := header.Unmarshal(data); err != nil {
		return 0, err
	}
	if header.Magic != DT_NAVMESH_MAGIC {
		return statusError(DT_FAILURE | DT_WRONG_MAGIC)
	}
	if header.Version != DT_NAVMESH_VERSION {
		return statusError(DT_FAILURE | DT_WRONG_VERSION)
	}

	// Make sure the location is free.
	if n.GetTileAt(header.X, header.Y, header.Layer) == nil {
		return statusError(DT_FAILURE)
	}

	// Allocate a tile.
	var tile *MeshTile
	if lastRef == 0 {
		if n.nextFree != nil {
			tile = n.nextFree
			n.nextFree, tile.Next = tile.Next, nil
		}
	} else {
		// Try to relocate the tile to specific index with same salt.
		tileIndex := n.DecodePolyIdTile(PolyRef(lastRef))
		if tileIndex >= n.maxTiles {
			return statusError(DT_FAILURE | DT_OUT_OF_MEMORY)
		}

		// Try to find the specific tile id from the free list.
		target := &n.tiles[tileIndex]
		var prev *MeshTile
		tile = n.nextFree
		for tile != nil && tile != target {
			prev, tile = tile, tile.Next
		}

		// Could not find the correct location.
		if tile != target {
			return statusError(DT_FAILURE | DT_OUT_OF_MEMORY)
		}

		// Remove from freelist
		if prev == nil {
			n.nextFree = tile.Next
		} else {
			prev.Next = tile.Next
		}

		// Restore salt.
		tile.Salt = n.DecodePolyIdSalt(PolyRef(lastRef))
	}

	// Make sure we could allocate a tile.
	if tile == nil {
		return statusError(DT_FAILURE | DT_OUT_OF_MEMORY)
	}

	// Insert tile into the position lut.
	h := computeTileHash(header.X, header.Y, n.tileLutMask)
	tile.Next, n.posLookup[h] = n.posLookup[h], tile

	// Patch header pointers.
	const (
		headerSize       = align4(sizeof(MeshHeader{}))
		vertsSize        = align4(sizeof(float32(0)) * 3 * header.VertCount)
		polysSize        = align4(sizeof(Poly{}) * header.PolyCount)
		linksSize        = align4(sizeof(Link{}) * (header.MaxLinkCount))
		detailMeshesSize = align4(sizeof(PolyDetail{}) * *header.DetailMeshCount)
		detailVertsSize  = align4(sizeof(float32(0)) * 3 * header.DetailVertCount)
		detailTrisSize   = align4(sizeof(uint8(0)) * 4 * header.DetailTriCount)
		bvtreeSize       = align4(sizeof(BVNode{}) * header.BVNodeCount)
		offMeshLinksSize = align4(sizeof(OffMeshConnection{}) * header.OffMeshConCount)
	)

	d := data[headerSize:]
	//tile.verts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	//tile->polys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);
	//tile->links = dtGetThenAdvanceBufferPointer<dtLink>(d, linksSize);
	//tile->detailMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
	//tile->detailVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	//tile->detailTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	//tile->bvTree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvtreeSize);
	//tile->offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(d, offMeshLinksSize);

	// If there are no items in the bvtree, reset the tree pointer.
	if bvtreeSize == 0 {
		tile.BVTree = nil
	}

	// Build links freelist
	tile.LinksFreeList = 0
	tile.Links[header.MaxLinkCount-1].next = DT_NULL_LINK
	for i := 0; i < header.MaxLinkCount-1; i++ {
		tile.Links[i].next = i + 1
	}

	// Init tile.
	tile.Header = header
	tile.Data = data
	tile.Flags = flags

	n.connectIntLinks(tile)

	// Base off-mesh connections to their starting polygons and connect connections inside the tile.
	baseOffMeshLinks(tile)
	connectExtOffMeshLinks(tile, tile, -1)

	// Create connections with neighbour tiles.
	const MAX_NEIS = 32

	// Connect with layers in current tile.
	neis := n.GetTilesAt(header.X, header.Y, MAX_NEIS)
	for j := 0; j < len(neis); j++ {
		if neis[j] == tile {
			continue
		}

		connectExtLinks(tile, neis[j], -1)
		connectExtLinks(neis[j], tile, -1)
		connectExtOffMeshLinks(tile, neis[j], -1)
		connectExtOffMeshLinks(neis[j], tile, -1)
	}

	// Connect with neighbour tiles.
	for i := 0; i < 8; i++ {
		neis = getNeighbourTilesAt(header.X, header.Y, i, MAX_NEIS)
		for j := 0; j < len(neis); j++ {
			connectExtLinks(tile, neis[j], i)
			connectExtLinks(neis[j], tile, dtOppositeTile(i))
			connectExtOffMeshLinks(tile, neis[j], i)
			connectExtOffMeshLinks(neis[j], tile, dtOppositeTile(i))
		}
	}

	if result {
		*result = getTileRef(tile)
	}

	return nil
}

// Removes the specified tile from the navigation mesh.
//  @param[in]		ref			The reference of the tile to remove.
// @return The Data associated with deleted tile and status flags for the operation.
func (n *NavMesh) RemoveTile(ref TileRef) ([]byte, error) {
	panic("implement")
}

// @}

// @{
// @name Query Functions

// Calculates the tile grid location for the specified world position.
//  @param[in]	pos  The world position for the query. [(x, y, z)]
//  @return	tx		The tile's x-location. (x, y)
//  @return	ty		The tile's y-location. (x, y)
func (n *NavMesh) CalcTileLoc(pos []float32) (tx, ty int) {
	panic("implement")
}

// Gets the tile at the specified grid location.
//  @param[in]	x		The tile's x-location. (x, y, layer)
//  @param[in]	y		The tile's y-location. (x, y, layer)
//  @param[in]	layer	The tile's layer. (x, y, layer)
// @return The tile, or null if the tile does not exist.
func (n *NavMesh) GetTileAt(x, y, layer int) *MeshTile {
	panic("implement")
}

// Gets all tiles at the specified grid location. (All layers.)
//  @param[in]		x			The tile's x-location. (x, y)
//  @param[in]		y			The tile's y-location. (x, y)
//  @param[in]		maxTiles	The maximum tiles the tiles parameter can hold.
// @return The tiles array.
func (n *NavMesh) GetTilesAt(x, y, maxTiles int) []*MeshTile {
	panic("implement")
}

// Gets the tile reference for the tile at specified grid location.
//  @param[in]	x		The tile's x-location. (x, y, layer)
//  @param[in]	y		The tile's y-location. (x, y, layer)
//  @param[in]	layer	The tile's layer. (x, y, layer)
// @return The tile reference of the tile, or 0 if there is none.
func (n *NavMesh) GetTileRefAt(x, y, layer int) TileRef {
	panic("implement")
}

// Gets the tile reference for the specified tile.
//  @param[in]	tile	The tile.
// @return The tile reference of the tile.
func (n NavMesh) GetTileRef(tile *MeshTile) TileRef {
	panic("implement")
}

// Gets the tile for the specified tile reference.
//  @param[in]	ref		The tile reference of the tile to retrieve.
// @return The tile for the specified reference, or null if the
//		reference is invalid.
func (n *NavMesh) GetTileByRef(ref TileRef) *MeshTile {
	panic("implement")
}

// The maximum number of tiles supported by the navigation mesh.
// @return The maximum number of tiles supported by the navigation mesh.
func (n *NavMesh) GetMaxTiles() int {
	panic("implement")
}

// Gets the tile at the specified index.
//  @param[in]	i		The tile index. [Limit: 0 >= index < #getMaxTiles()]
// @return The tile at the specified index.
func (n *NavMesh) GetTile(i int) *MeshTile {
	if i < 0 || i >= len(n.tiles) {
		return nil
	}

	return &n.tiles[i]
}

// Gets the tile and polygon for the specified polygon reference.
//  @param[in]		ref		The reference for the a polygon.
//  @param[out]	tile	The tile containing the polygon.
//  @param[out]	poly	The polygon.
// @return The status flags for the operation.
func (n *NavMesh) GetTileAndPolyByRef(ref PolyRef) (*MeshTile, *Poly, error) {
	panic("implement")
}

// Checks the validity of a polygon reference.
//  @param[in]	ref		The polygon reference to check.
// @return True if polygon reference is valid for the navigation mesh.
func (n *NavMesh) IsValidPolyRef(ref PolyRef) bool {
	panic("implement")
}

// Gets the polygon reference for the tile's base polygon.
//  @param[in]	tile		The tile.
// @return The polygon reference for the base polygon in the specified tile.
func (n *NavMesh) GetPolyRefBase(tile *MeshTile) PolyRef {
	panic("implement")
}

// Gets the endpoints for an off-mesh connection, ordered by "direction of travel".
//  @param[in]		prevRef		The reference of the polygon before the connection.
//  @param[in]		polyRef		The reference of the off-mesh connection polygon.
//  @param[out]	startPos	The start position of the off-mesh connection. [(x, y, z)]
//  @param[out]	endPos		The end position of the off-mesh connection. [(x, y, z)]
// @return The status flags for the operation.
func (n *NavMesh) GetOffMeshConnectionPolyEndPoints(prefRef PolyRef, polyRef PolyRef) (startPos, endPos float32, _ error) {
	panic("implement")
}

// Gets the specified off-mesh connection.
//  @param[in]	ref		The polygon reference of the off-mesh connection.
// @return The specified off-mesh connection, or null if the polygon reference is not valid.
func (n *NavMesh) GetOffMeshConnectionByRef(ref PolyRef) (*OffMeshConnection, error) {
	panic("implement")
}

// @}

// @{
// @name State Management
// These functions do not effect #dtTileRef or #dtPolyRef's.

// Sets the user defined flags for the specified polygon.
//  @param[in]	ref		The polygon reference.
//  @param[in]	flags	The new flags for the polygon.
// @return The status flags for the operation.
func (n *NavMesh) SetPolyFlags(ref PolyRef, flags uint16) error {
	panic("implement")
}

// Gets the user defined flags for the specified polygon.
//  @param[in]		ref				The polygon reference.
//  @param[out]	resultFlags		The polygon flags.
// @return The status flags for the operation.
func (n *NavMesh) GetPolyFlags(ref PolyRef) (uint16, error) {
	panic("implement")
}

// Sets the user defined area for the specified polygon.
//  @param[in]	ref		The polygon reference.
//  @param[in]	area	The new area id for the polygon. [Limit: < #DT_MAX_AREAS]
// @return The status flags for the operation.
func (n *NavMesh) SetPolyArea(ref PolyRef, area uint8) error {
	panic("implement")
}

// Gets the user defined area for the specified polygon.
//  @param[in]		ref			The polygon reference.
//  @param[out]	resultArea	The area id for the polygon.
// @return The status flags for the operation.
func (n *NavMesh) GetPolyArea(ref PolyRef) (uint8, error) {
	panic("implement")
}

// Gets the size of the buffer required by #storeTileState to store the specified tile's state.
//  @param[in]	tile	The tile.
// @return The size of the buffer required to store the state.
func (n *NavMesh) GetTileStateSize(tile *MeshTile) int {
	panic("implement")
}

// Stores the non-structural state of the tile in the specified buffer. (Flags, area ids, etc.)
//  @param[in]		tile			The tile.
//  @param[out]	data			The buffer to store the tile's state in.
//  @param[in]		maxDataSize		The size of the data buffer. [Limit: >= #getTileStateSize]
// @return The status flags for the operation.
func (n *NavMesh) StoreTileState(tile *MeshTile, maxDataSize int) ([]byte, error) {
	panic("implement")
}

// Restores the state of the tile.
//  @param[in]	tile			The tile.
//  @param[in]	data			The new state. (Obtained from #storeTileState.)
// @return The status flags for the operation.
func (n *NavMesh) RestoreTileState(tile *MeshTile, data []byte) error {
	panic("implement")
}

// @}

// @{
// @name Encoding and Decoding
// These functions are generally meant for internal use only.

// Derives a standard polygon reference.
//  @note This function is generally meant for internal use only.
//  @param[in]	salt	The tile's salt value.
//  @param[in]	it		The index of the tile.
//  @param[in]	ip		The index of the polygon within the tile.
func (n *NavMesh) EncodePolyId(salt, it, ip uint) PolyRef {
	//#ifdef DT_POLYREF64
	//return ((dtPolyRef)salt << (DT_POLY_BITS+DT_TILE_BITS)) | ((dtPolyRef)it << DT_POLY_BITS) | (dtPolyRef)ip;
	//#else
	return PolyRef((salt << (n.polyBits + n.tileBits)) | (it << n.polyBits) | ip)
	//#endif
}

// Decodes a standard polygon reference.
//  @note This function is generally meant for internal use only.
//  @param[in]	ref   The polygon reference to decode.
//  @param[out]	salt	The tile's salt value.
//  @param[out]	it		The index of the tile.
//  @param[out]	ip		The index of the polygon within the tile.
//  @see #encodePolyId
func (n *NavMesh) DecodePolyId(ref PolyRef) (salt, it, ip uint) {
	//#ifdef DT_POLYREF64
	//const dtPolyRef saltMask = ((dtPolyRef)1<<DT_SALT_BITS)-1;
	//const dtPolyRef tileMask = ((dtPolyRef)1<<DT_TILE_BITS)-1;
	//const dtPolyRef polyMask = ((dtPolyRef)1<<DT_POLY_BITS)-1;
	//salt = (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
	//it = (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
	//ip = (unsigned int)(ref & polyMask);
	//#else
	saltMask := uint(1<<n.saltBits) - 1
	tileMask := uint(1<<n.tileBits) - 1
	polyMask := uint(1<<n.polyBits) - 1
	salt = ((uint(ref) >> (n.polyBits + n.tileBits)) & saltMask)
	it = ((uint(ref) >> n.polyBits) & tileMask)
	ip = (uint(ref) & polyMask)
	//#endif

	return
}

// Extracts a tile's salt value from the specified polygon reference.
//  @note This function is generally meant for internal use only.
//  @param[in]	ref		The polygon reference.
//  @see #encodePolyId
func (n *NavMesh) DecodePolyIdSalt(ref PolyRef) uint {
	//#ifdef DT_POLYREF64
	//const dtPolyRef saltMask = ((dtPolyRef)1<<DT_SALT_BITS)-1;
	//return (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
	//#else
	saltMask := uint(1<<n.saltBits) - 1
	return ((uint(ref) >> (n.polyBits + n.tileBits)) & saltMask)
	//#endif
}

// Extracts the tile's index from the specified polygon reference.
//  @note This function is generally meant for internal use only.
//  @param[in]	ref		The polygon reference.
//  @see #encodePolyId
func (n *NavMesh) DecodePolyIdTile(ref PolyRef) uint {
	//#ifdef DT_POLYREF64
	//const dtPolyRef tileMask = ((dtPolyRef)1<<DT_TILE_BITS)-1;
	//return (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
	//#else
	tileMask := uint(1<<n.tileBits) - 1
	return ((uint(ref) >> n.polyBits) & tileMask)
	//#endif
}

// Extracts the polygon's index (within its tile) from the specified polygon reference.
//  @note This function is generally meant for internal use only.
//  @param[in]	ref		The polygon reference.
//  @see #encodePolyId
func (n *NavMesh) DecodePolyIdPoly(ref PolyRef) uint {
	//#ifdef DT_POLYREF64
	//const dtPolyRef polyMask = ((dtPolyRef)1<<DT_POLY_BITS)-1;
	//return (unsigned int)(ref & polyMask);
	//#else
	polyMask := uint(1<<n.polyBits) - 1
	return (uint(ref) & polyMask)
	//#endif
}

func (n *NavMesh) connectIntLinks(tile *MeshTile) {
	if tile == nil {
		return
	}

	base := n.GetPolyRefBase(tile)
	for i := 0; i < tile.Header.PolyCount; i++ {
		poly := tile.Polys[i]
		poly.FirstLink = DT_NULL_LINK

		if poly.GetType() == DT_POLYTYPE_OFFMESH_CONNECTION {
			continue
		}

		// Build edge links backwards so that the links will be
		// in the linked list from lowest index to highest.
		for j := poly.VertCount - 1; j >= 0; j-- {
			// Skip hard and non-internal edges.
			if poly.Neis[j] == 0 || (poly.Neis[j]&DT_EXT_LINK) != 0 {
				continue
			}

			idx := allocLink(tile)
			if idx != DT_NULL_LINK {
				link := &tile.Links[idx]
				link.ref = base | PolyRef(poly.Neis[j]-1)
				link.edge = uint8(j)
				link.side = 0xff
				link.bmin, link.bmax = 0, 0
				// Add to linked list.
				link.next, poly.FirstLink = poly.FirstLink, idx
			}
		}
	}
}

func (n *NavMesh) queryPolygonsInTile(tile *MeshTile, qmin, qmax mgl32.Vec3,  polys[]PolyRef) int {
	const maxPolys = len(polys)
	if len(tile.BVTree) != 0 {
		node := &tile.BVTree[0];
		end := &tile.BVTree[tile.Header.BVNodeCount];
		tbmin := tile.Header.BMin;
		tbmax := tile.Header.BMax;
		qfac := tile.Header.BVQuantFactor;

		// Calculate quantized box
		var bmin, bmax[3] uint16;
		// dtClamp query box to world box.
		minx := clamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
		miny := clamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
		minz := clamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
		maxx := clamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
		maxy := clamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
		maxz := clamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
		// Quantize
		bmin[0] = uint16(qfac * minx) & 0xfffe;
		bmin[1] = uint16(qfac * miny) & 0xfffe;
		bmin[2] = uint16(qfac * minz) & 0xfffe;
		bmax[0] = uint16(qfac * maxx + 1) | 1;
		bmax[1] = uint16(qfac * maxy + 1) | 1;
		bmax[2] = uint16(qfac * maxz + 1) | 1;

		// Traverse tree
		base := n.GetPolyRefBase(tile);
		ret := 0;
		for node != end {
			const overlap = dtOverlapQuantBounds(bmin, bmax, node.bmin, node.bmax);
			const isLeafNode = node.i >= 0;

			if isLeafNode && overlap {
				if n < maxPolys {
					polys[n] = base | PolyRef(node.i);
					ret++
				}
			}

			if overlap || isLeafNode {
				node++;
			} else {
				const escapeIndex = -node.i;
				node += escapeIndex;
			}
		}

		return ret;
	} else {
		var bmin, bmax mgl32.Vec3
		ret := 0;
		base := n.GetPolyRefBase(tile);
		for i := 0; i < tile.Header.PolyCount; i++ {
			p := &tile.Polys[i];
			// Do not return off-mesh connection polygons.
			if (p.GetType() == DT_POLYTYPE_OFFMESH_CONNECTION) {
				continue;
			}
			// Calc polygon bounds.
			v := &tile.Verts[p.Verts[0]*3];
			copy(bmin[:], v[:])
			copy(bmax[:], v[:])
			for j := 1; j < p.VertCount; j++{
				v = &tile.Verts[p.Verts[j]*3];
				minv(bmin, v);
				maxv(bmax, v);
			}
			if dtOverlapBounds(qmin,qmax, bmin,bmax){
			if (n < maxPolys) {
				polys[n] = base | PolyRef( i);
				ret++
			}
			}
		}
		return ret;
	}
}

func (n *NavMesh) findNearestPolyInTile(tile *MeshTile, center, extents , nearestPt mgl32.Vec3) PolyRef {
	bmin := center.Sub(extents)
	bmax := center.Add(extents)

	polys := n.queryPolygonsInTile()
	// Get nearby polygons from proximity grid.
	dtPolyRef polys[128];
	int polyCount = queryPolygonsInTile(tile, bmin, bmax, polys, 128);

// Find nearest polygon amongst the nearby polygons.
dtPolyRef nearest = 0;
float nearestDistanceSqr = FLT_MAX;
for (int i = 0; i < polyCount; ++i)
{
dtPolyRef ref = polys[i];
float closestPtPoly[3];
float diff[3];
bool posOverPoly = false;
float d;
closestPointOnPoly(ref, center, closestPtPoly, &posOverPoly);

// If a point is directly over a polygon and closer than
// climb height, favor that instead of straight line nearest point.
dtVsub(diff, center, closestPtPoly);
if (posOverPoly)
{
d = dtAbs(diff[1]) - tile->header->walkableClimb;
d = d > 0 ? d*d : 0;
}
else
{
d = dtVlenSqr(diff);
}

if (d < nearestDistanceSqr)
{
dtVcopy(nearestPt, closestPtPoly);
nearestDistanceSqr = d;
nearest = ref;
}
}

return nearest;
}


func (n *NavMesh) baseOffMeshLinks(tile *MeshTile) {
	if tile == nil {
		return
	}

	base := n.GetPolyRefBase(tile)

// Base off-mesh connection start points.
	for i := 0; i < tile.Header.OffMeshConCount; i++ {
		con := tile.OffMeshCons[i]
		poly := tile.Polys[con.Poly]

		const ext = [3]float32{ con.Rad, tile.Header.WalkableClimb, con.Rad};

		// Find polygon to connect to.
		const p = &con.Pos[0]; // First vertex
		float nearestPt[3];
		dtPolyRef ref = findNearestPolyInTile(tile, p, ext, nearestPt);
		if (!ref) continue;
		// findNearestPoly may return too optimistic results, further check to make sure.
		if (dtSqr(nearestPt[0]-p[0])+dtSqr(nearestPt[2]-p[2]) > dtSqr(con->rad))
		continue;
		// Make sure the location is on current mesh.
		float* v = &tile->verts[poly->verts[0]*3];
		dtVcopy(v, nearestPt);

		// Link off-mesh connection to target poly.
		unsigned int idx = allocLink(tile);
		if (idx != DT_NULL_LINK)
		{
		dtLink* link = &tile->links[idx];
		link->ref = ref;
		link->edge = (unsigned char)0;
		link->side = 0xff;
		link->bmin = link->bmax = 0;
		// Add to linked list.
		link->next = poly->firstLink;
		poly->firstLink = idx;
		}

		// Start end-point is always connect back to off-mesh connection.
		unsigned int tidx = allocLink(tile);
		if (tidx != DT_NULL_LINK)
		{
		const unsigned short landPolyIdx = (unsigned short)decodePolyIdPoly(ref);
		dtPoly* landPoly = &tile->polys[landPolyIdx];
		dtLink* link = &tile->links[tidx];
		link->ref = base | (dtPolyRef)(con->poly);
		link->edge = 0xff;
		link->side = 0xff;
		link->bmin = link->bmax = 0;
		// Add to linked list.
		link->next = landPoly->firstLink;
		landPoly->firstLink = tidx;
		}
	}
}


// @}

func computeTileHash(x, y, mask int) int {
	const h1 = uint(0x8da6b343) // Large multiplicative constants;
	const h2 = uint(0xd8163841) // here arbitrarily chosen primes
	n := h1*uint(x) + h2*uint(y)
	return int(n & mask)
}

func allocLink(tile *MeshTile) uint {
	if tile.LinksFreeList == DT_NULL_LINK {
		return DT_NULL_LINK
	}

	link := tile.LinksFreeList
	tile.LinksFreeList = tile.Links[link].next
	return link
}
