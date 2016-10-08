package detour

import (
	"bytes"
	"io"
	"math"
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

func (p *Poly) Parse(r io.Reader) (err error) {
	reader := UtilReader{r: r}
	if p.FirstLink, err = reader.ReadUint(); err != nil {
		return err
	}
	for i := range p.Verts {
		if p.Verts[i], err = reader.ReadUint16(); err != nil {
			return err
		}
	}
	for i := range p.Neis {
		if p.Neis[i], err = reader.ReadUint16(); err != nil {
			return err
		}
	}
	if p.Flags, err = reader.ReadUint16(); err != nil {
		return err
	}
	if p.VertCount, err = reader.ReadUint8(); err != nil {
		return err
	}
	if p.areaAndtype, err = reader.ReadUint8(); err != nil {
		return err
	}
	return nil
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

func (p *PolyDetail) Parse(r io.Reader) (err error) {
	reader := UtilReader{r: r}
	if p.VertBase, err = reader.ReadUint(); err != nil {
		return err
	}
	if p.TriBase, err = reader.ReadUint(); err != nil {
		return err
	}
	if p.VertCount, err = reader.ReadUint8(); err != nil {
		return err
	}
	if p.TriCount, err = reader.ReadUint8(); err != nil {
		return err
	}
	return nil
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

func (l *Link) Parse(r io.Reader) (err error) {
	reader := UtilReader{r: r}
	if l.ref, err = reader.ReadUint(); err != nil {
		return err
	}
	if l.next, err = reader.ReadUint(); err != nil {
		return err
	}
	if l.edge, err = reader.ReadUint8(); err != nil {
		return err
	}
	if l.side, err = reader.ReadUint8(); err != nil {
		return err
	}
	if l.bmin, err = reader.ReadUint8(); err != nil {
		return err
	}
	if l.bmax, err = reader.ReadUint8(); err != nil {
		return err
	}
	return nil
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

	DetailTriCount  int     //< The number of triangles in the detail mesh.
	BVNodeCount     int     //< The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
	OffMeshConCount int     //< The number of off-mesh connections.
	OffMeshBase     int     //< The index of the first polygon which is an off-mesh connection.
	WalkableHeight  float32 //< The height of the agents using the tile.
	WalkableRadius  float32 //< The radius of the agents using the tile.
	WalkableClimb   float32 //< The maximum climb height of the agents using the tile.
	BMin            Vector3 //< The minimum bounds of the tile's AABB. [(x, y, z)]
	BMax            Vector3 //< The maximum bounds of the tile's AABB. [(x, y, z)]

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
	Verts         []Vector3    //< The tile vertices. [Size: dtMeshHeader::vertCount]
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
	Orig       Vector3 //< The world space origin of the navigation mesh's tile space. [(x, y, z)]
	TileWidth  float32 //< The width of each tile. (Along the x-axis.)
	TileHeight float32 //< The height of each tile. (Along the z-axis.)
	MaxTiles   uint    //< The maximum number of tiles the navigation mesh can contain.
	MaxPolys   uint    //< The maximum number of polygons each tile can contain.
}

// A navigation mesh based on tiles of convex polygons.
type NavMesh struct {
	params                NavMeshParams //< Current initialization params. TODO: do not store this info twice.
	orig                  Vector3       //< Origin of the tile (0,0)
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
	n.orig.CopyFrom(params.Orig)
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
	params.Orig.CopyFrom(header.BMin)
	params.TileWidth = header.BMax[0] - header.BMin[0]
	params.TileHeight = header.BMax[2] - header.BMin[2]
	params.MaxTiles = 1
	params.MaxPolys = uint(header.PolyCount)

	if err := n.InitWithParams(params); err != nil {
		return err
	}

	_, err := n.AddTile(data, flags, 0)
	return err
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
		return 0, statusError(DT_FAILURE | DT_WRONG_MAGIC)
	}
	if header.Version != DT_NAVMESH_VERSION {
		return 0, statusError(DT_FAILURE | DT_WRONG_VERSION)
	}

	// Make sure the location is free.
	if n.GetTileAt(header.X, header.Y, header.Layer) == nil {
		return 0, statusError(DT_FAILURE)
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
		tileIndex := int(n.DecodePolyIdTile(PolyRef(lastRef)))
		if tileIndex >= n.maxTiles {
			return 0, statusError(DT_FAILURE | DT_OUT_OF_MEMORY)
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
			return 0, statusError(DT_FAILURE | DT_OUT_OF_MEMORY)
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
		return 0, statusError(DT_FAILURE | DT_OUT_OF_MEMORY)
	}

	// Insert tile into the position lut.
	h := computeTileHash(header.X, header.Y, n.tileLutMask)
	tile.Next, n.posLookup[h] = n.posLookup[h], tile

	// Patch header pointers.
	var (
		headerSize = align4(sizeof(MeshHeader{}))
		//vertsSize        = align4(sizeof(float32(0)) * 3 * header.VertCount)
		//polysSize        = align4(sizeof(Poly{}) * header.PolyCount)
		//linksSize        = align4(sizeof(Link{}) * (header.MaxLinkCount))
		//detailMeshesSize = align4(sizeof(PolyDetail{}) * header.DetailMeshCount)
		//detailVertsSize  = align4(sizeof(float32(0)) * 3 * header.DetailVertCount)
		//detailTrisSize   = align4(sizeof(uint8(0)) * 4 * header.DetailTriCount)
		bvtreeSize = align4(sizeof(BVNode{}) * header.BVNodeCount)
		//offMeshLinksSize = align4(sizeof(OffMeshConnection{}) * header.OffMeshConCount)
	)

	d := bytes.NewBuffer(data[headerSize:])
	//tile.verts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	tile.Verts = make([]Vector3, header.VertCount)
	for i := range tile.Verts {
		if err := tile.Verts[i].Parse(d); err != nil {
			return 0, err
		}
	}

	//tile->polys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);
	tile.Polys = make([]Poly, header.PolyCount)
	for i := range tile.Polys {
		if err := tile.Polys[i].Parse(d); err != nil {
			return 0, err
		}
	}

	//tile->links = dtGetThenAdvanceBufferPointer<dtLink>(d, linksSize);
	tile.Links = make([]Link, header.MaxLinkCount)
	for i := range tile.Links {
		if err := tile.Links[i].Parse(d); err != nil {
			return 0, err
		}
	}

	//tile->detailMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
	tile.DetailMeshes = make([]PolyDetail, header.DetailMeshCount)
	for i := range tile.DetailMeshes {
		if err := tile.DetailMeshes[i].Parse(d); err != nil {
			return 0, err
		}
	}

	//tile->detailVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	tile.DetailVerts = make([]float32, header.DetailVertCount)
	for i := range tile.DetailMeshes {
		if err := tile.DetailMeshes[i].Parse(d); err != nil {
			return 0, err
		}
	}

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
		tile.Links[i].next = uint(i + 1)
	}

	// Init tile.
	tile.Header = &header
	tile.Data = data
	tile.Flags = flags

	n.connectIntLinks(tile)

	// Base off-mesh connections to their starting polygons and connect connections inside the tile.
	n.baseOffMeshLinks(tile)
	n.connectExtOffMeshLinks(tile, tile, -1)

	// Create connections with neighbour tiles.
	const MAX_NEIS = 32
	var neis [MAX_NEIS]*MeshTile

	// Connect with layers in current tile.
	nneis := n.GetTilesAt(header.X, header.Y, neis[:])
	for j := 0; j < nneis; j++ {
		if neis[j] == tile {
			continue
		}

		n.connectExtLinks(tile, neis[j], -1)
		n.connectExtLinks(neis[j], tile, -1)
		n.connectExtOffMeshLinks(tile, neis[j], -1)
		n.connectExtOffMeshLinks(neis[j], tile, -1)
	}

	// Connect with neighbour tiles.
	for i := 0; i < 8; i++ {
		nneis := n.getNeighbourTilesAt(header.X, header.Y, i, neis[:])
		for j := 0; j < nneis; j++ {
			n.connectExtLinks(tile, neis[j], i)
			n.connectExtLinks(neis[j], tile, oppositeTile(i))
			n.connectExtOffMeshLinks(tile, neis[j], i)
			n.connectExtOffMeshLinks(neis[j], tile, oppositeTile(i))
		}
	}

	return n.GetTileRef(tile), nil
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
func (n *NavMesh) GetTilesAt(x, y int, tiles []*MeshTile) int {
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

func (n *NavMesh) queryPolygonsInTile(tile *MeshTile, qmin, qmax Vector3, polys []PolyRef) int {
	maxPolys := len(polys)
	if len(tile.BVTree) != 0 {
		tbmin := tile.Header.BMin
		tbmax := tile.Header.BMax
		qfac := tile.Header.BVQuantFactor

		// Calculate quantized box
		var bmin, bmax [3]uint16
		// dtClamp query box to world box.
		minx := clamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0]
		miny := clamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1]
		minz := clamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2]
		maxx := clamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0]
		maxy := clamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1]
		maxz := clamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2]
		// Quantize
		bmin[0] = uint16(qfac*minx) & 0xfffe
		bmin[1] = uint16(qfac*miny) & 0xfffe
		bmin[2] = uint16(qfac*minz) & 0xfffe
		bmax[0] = uint16(qfac*maxx+1) | 1
		bmax[1] = uint16(qfac*maxy+1) | 1
		bmax[2] = uint16(qfac*maxz+1) | 1

		// Traverse tree
		base := n.GetPolyRefBase(tile)
		ret := 0
		for i := 0; i < tile.Header.BVNodeCount; {
			node := &tile.BVTree[i]
			overlap := overlapQuantBounds(bmin, bmax, node.bmin, node.bmax)
			isLeafNode := node.i >= 0

			if isLeafNode && overlap {
				if ret < maxPolys {
					polys[ret] = base | PolyRef(node.i)
					ret++
				}
			}

			if overlap || isLeafNode {
				i++
			} else {
				i += -node.i
			}
		}

		return ret
	} else {
		var bmin, bmax Vector3
		ret := 0
		base := n.GetPolyRefBase(tile)
		for i := 0; i < tile.Header.PolyCount; i++ {
			p := &tile.Polys[i]
			// Do not return off-mesh connection polygons.
			if p.GetType() == DT_POLYTYPE_OFFMESH_CONNECTION {
				continue
			}
			// Calc polygon bounds.
			v := NewVector3(tile.Verts[p.Verts[0]*3])
			bmin.CopyFrom(v)
			bmax.CopyFrom(v)
			for j := uint8(1); j < p.VertCount; j++ {
				v = tile.Verts[p.Verts[j]]
				vmin(bmin, v)
				vmax(bmax, v)
			}
			if overlapBounds(qmin, qmax, bmin, bmax) {
				if ret < maxPolys {
					polys[ret] = base | PolyRef(i)
					ret++
				}
			}
		}
		return ret
	}
}

func (n *NavMesh) findNearestPolyInTile(tile *MeshTile, center, extents Vector3) (nearest PolyRef, nearestPt Vector3) {
	bmin := center.Sub(extents)
	bmax := center.Add(extents)

	// Get nearby polygons from proximity grid.
	var polys [128]PolyRef
	polyCount := n.queryPolygonsInTile(tile, bmin, bmax, polys[:])

	// Find nearest polygon amongst the nearby polygons.
	nearestDistanceSqr := float32(math.MaxFloat32)
	for i := 0; i < polyCount; i++ {
		ref := polys[i]
		var d float32
		closestPtPoly, posOverPoly := n.closestPointOnPoly(ref, center)

		// If a point is directly over a polygon and closer than
		// climb height, favor that instead of straight line nearest point.
		diff := center.Sub(closestPtPoly)
		if posOverPoly {
			d = abs(diff[1]) - tile.Header.WalkableClimb
			if d > 0 {
				d = d * d
			} else {
				d = 0
			}
		} else {
			d = diff.Dot(diff)
		}

		if d < nearestDistanceSqr {
			nearestPt.CopyFrom(closestPtPoly)
			nearestDistanceSqr = d
			nearest = ref
		}
	}

	return nearest, nearestPt
}

func (n *NavMesh) baseOffMeshLinks(tile *MeshTile) {
	if tile == nil {
		return
	}

	base := n.GetPolyRefBase(tile)

	// Base off-mesh connection start points.
	for i := 0; i < tile.Header.OffMeshConCount; i++ {
		con := &tile.OffMeshCons[i]
		poly := &tile.Polys[con.Poly]

		ext := Vector3{con.Rad, tile.Header.WalkableClimb, con.Rad}

		// Find polygon to connect to.
		p := NewVector3(con.Pos[:]) // First vertex
		ref, nearestPt := n.findNearestPolyInTile(tile, p, ext)
		if ref == 0 {
			continue
		}

		// findNearestPoly may return too optimistic results, further check to make sure.
		if sqr(nearestPt[0]-p[0])+sqr(nearestPt[2]-p[2]) > sqr(con.Rad) {
			continue
		}

		// Make sure the location is on current mesh.
		v := tile.Verts[poly.Verts[0]]
		v.CopyFrom(nearestPt)

		// Link off-mesh connection to target poly.
		idx := allocLink(tile)
		if idx != DT_NULL_LINK {
			link := &tile.Links[idx]
			link.ref = ref
			link.edge = 0
			link.side = 0xff
			link.bmin, link.bmax = 0, 0
			// Add to linked list.
			link.next, poly.FirstLink = poly.FirstLink, idx
		}

		// Start end-point is always connect back to off-mesh connection.
		tidx := allocLink(tile)
		if tidx != DT_NULL_LINK {
			landPolyIdx := n.DecodePolyIdPoly(ref)
			landPoly := &tile.Polys[landPolyIdx]
			link := &tile.Links[tidx]
			link.ref = base | PolyRef(con.Poly)
			link.edge = 0xff
			link.side = 0xff
			link.bmin, link.bmax = 0, 0
			// Add to linked list.
			link.next, landPoly.FirstLink = landPoly.FirstLink, tidx
		}
	}
}

func (n *NavMesh) connectExtOffMeshLinks(tile, target *MeshTile, side int) {
	if tile == nil {
		return
	}

	// Connect off-mesh links.
	// We are interested on links which land from target tile to this tile.
	var oppositeSide uint8 = 0xff
	if side != -1 {
		oppositeSide = uint8(oppositeTile(side))
	}

	for i := 0; i < target.Header.OffMeshConCount; i++ {
		targetCon := &target.OffMeshCons[i]
		if targetCon.Side != oppositeSide {
			continue
		}

		targetPoly := &target.Polys[targetCon.Poly]
		// Skip off-mesh connections which start location could not be connected at all.
		if targetPoly.FirstLink == DT_NULL_LINK {
			continue
		}

		ext := Vector3{targetCon.Rad, target.Header.WalkableClimb, targetCon.Rad}

		// Find polygon to connect to.
		p := NewVector3(targetCon.Pos[3:])
		ref, nearestPt := n.findNearestPolyInTile(tile, p, ext)
		if ref == 0 {
			continue
		}

		// findNearestPoly may return too optimistic results, further check to make sure.
		if sqr(nearestPt[0]-p[0])+sqr(nearestPt[2]-p[2]) > sqr(targetCon.Rad) {
			continue
		}

		// Make sure the location is on current mesh.
		v := NewVector3(target.Verts[targetPoly.Verts[1]])
		v.CopyFrom(nearestPt)

		// Link off-mesh connection to target poly.
		idx := allocLink(target)
		if idx != DT_NULL_LINK {
			link := &target.Links[idx]
			link.ref = ref
			link.edge = 1
			link.side = oppositeSide
			link.bmin, link.bmax = 0, 0
			// Add to linked list.
			link.next, targetPoly.FirstLink = targetPoly.FirstLink, idx
		}

		// Link target poly to off-mesh connection.
		if targetCon.Flags&DT_OFFMESH_CON_BIDIR != 0 {
			tidx := allocLink(tile)
			if tidx != DT_NULL_LINK {
				landPolyIdx := n.DecodePolyIdPoly(ref)
				landPoly := &tile.Polys[landPolyIdx]
				link := &tile.Links[tidx]
				link.ref = n.GetPolyRefBase(target) | PolyRef(targetCon.Poly)
				link.edge = 0xff
				link.side = 0xff
				if side != -1 {
					link.side = uint8(side)
				}
				link.bmin, link.bmax = 0, 0
				// Add to linked list.
				link.next, landPoly.FirstLink = landPoly.FirstLink, tidx
			}
		}
	}

}

func (n *NavMesh) connectExtLinks(tile, target *MeshTile, side int) {
	if tile == nil {
		return
	}

	// Connect border links.
	for i := 0; i < tile.Header.PolyCount; i++ {
		poly := &tile.Polys[i]

		// Create new links.
		//		unsigned short m = DT_EXT_LINK | (unsigned short)side;
		nv := poly.VertCount
		for j := uint8(0); j < nv; j++ {
			// Skip non-portal edges.
			if (poly.Neis[j] & DT_EXT_LINK) == 0 {
				continue
			}

			dir := int(poly.Neis[j] & 0xff)
			if side != -1 && dir != side {
				continue
			}

			// Create new links
			va := tile.Verts[poly.Verts[j]]
			vb := tile.Verts[poly.Verts[(j+1)%nv]]
			var nei [4]PolyRef
			var neia [4 * 2]float32
			nnei := n.findConnectingPolys(va, vb, target, oppositeTile(dir), nei[:], neia[:], 4)
			for k := 0; k < nnei; k++ {
				idx := allocLink(tile)
				if idx != DT_NULL_LINK {
					link := &tile.Links[idx]
					link.ref = nei[k]
					link.edge = j
					link.side = uint8(dir)

					link.next, poly.FirstLink = poly.FirstLink, idx

					// Compress portal limits to a byte value.
					if dir == 0 || dir == 4 {
						tmin := (neia[k*2+0] - va[2]) / (vb[2] - va[2])
						tmax := (neia[k*2+1] - va[2]) / (vb[2] - va[2])
						if tmin > tmax {
							tmin, tmax = tmax, tmin
						}
						link.bmin = uint8(clamp(tmin, 0, 1) * 255)
						link.bmax = uint8(clamp(tmax, 0, 1) * 255)
					} else if dir == 2 || dir == 6 {
						tmin := (neia[k*2+0] - va[0]) / (vb[0] - va[0])
						tmax := (neia[k*2+1] - va[0]) / (vb[0] - va[0])
						if tmin > tmax {
							tmin, tmax = tmax, tmin
						}

						link.bmin = uint8(clamp(tmin, 0, 1) * 255)
						link.bmax = uint8(clamp(tmax, 0, 1) * 255)
					}
				}
			}
		}
	}
}

func (n *NavMesh) getNeighbourTilesAt(x, y, side int, tiles []*MeshTile) int {
	nx, ny := x, y
	switch side {
	case 0:
		nx++
	case 1:
		nx++
		ny++
	case 2:
		ny++
	case 3:
		nx--
		ny++
	case 4:
		nx--
	case 5:
		nx--
		ny--
	case 6:
		ny--
	case 7:
		nx++
		ny--
	}

	return n.GetTilesAt(nx, ny, tiles)
}

func (navMesh *NavMesh) findConnectingPolys(va, vb Vector3, tile *MeshTile, side int, con []PolyRef, conarea []float32, maxcon int) int {
	if tile == nil {
		return 0
	}

	amin, amax := calcSlabEndPoints(va, vb, side)
	apos := getSlabCoord(va, side)

	// Remove links pointing to 'side' and compact the links array.
	m := uint16(DT_EXT_LINK | side)
	n := 0

	base := navMesh.GetPolyRefBase(tile)

	for i := 0; i < tile.Header.PolyCount; i++ {
		poly := &tile.Polys[i]
		nv := poly.VertCount
		for j := uint8(0); j < nv; j++ {
			// Skip edges which do not point to the right side.
			if poly.Neis[j] != m {
				continue
			}

			vc := tile.Verts[poly.Verts[j]]
			vd := tile.Verts[poly.Verts[(j+1)%nv]]
			bpos := getSlabCoord(vc, side)

			// Segments are not close enough.
			if abs(apos-bpos) > 0.01 {
				continue
			}

			// Check if the segments touch.
			bmin, bmax := calcSlabEndPoints(vc, vd, side)

			if !overlapSlabs(amin, amax, bmin, bmax, 0.01, tile.Header.WalkableClimb) {
				continue
			}

			// Add return value.
			if n < maxcon {
				conarea[n*2+0] = fmax(amin[0], bmin[0])
				conarea[n*2+1] = fmin(amax[0], bmax[0])
				con[n] = base | PolyRef(i)
				n++
			}
			break
		}
	}
	return n
}

func (n *NavMesh) closestPointOnPoly(ref PolyRef, pos Vector3) (closest Vector3, posOverPoly bool) {
	tile, poly, _ := n.GetTileAndPolyByRef(ref)

	// Off-mesh connections don't have detail polygons.
	if poly.GetType() == DT_POLYTYPE_OFFMESH_CONNECTION {
		v0 := tile.Verts[poly.Verts[0]]
		v1 := tile.Verts[poly.Verts[1]]
		d0 := pos.DistanceTo(v0)
		d1 := pos.DistanceTo(v1)
		u := d0 / (d0 + d1)
		closest = vlerp(v0, v1, u)
		posOverPoly = false
		return
	}

	var pd *PolyDetail
	for i := range tile.Polys {
		if &tile.Polys[i] == poly {
			pd = &tile.DetailMeshes[i]
			break
		}
	}

	// Clamp point to be inside the polygon.
	var verts [DT_VERTS_PER_POLYGON]Vector3
	var edged [DT_VERTS_PER_POLYGON]float32
	var edget [DT_VERTS_PER_POLYGON]float32
	nv := poly.VertCount
	for i := 0; i < int(nv); i++ {
		verts[i].CopyFrom(tile.Verts[poly.Verts[i]])
	}

	closest.CopyFrom(pos)
	if !distancePtPolyEdgesSqr(pos, verts[:], int(nv), edged[:], edget[:]) {
		// Point is outside the polygon, dtClamp to nearest edge.
		dmin := edged[0]
		imin := uint8(0)
		for i := uint8(1); i < nv; i++ {
			if edged[i] < dmin {
				dmin = edged[i]
				imin = i
			}
		}
		va := verts[imin]
		vb := verts[((imin + 1) % nv)]
		closest = vlerp(va, vb, edget[imin])
		posOverPoly = false
	} else {
		posOverPoly = true
	}

	// Find height at the location.
	for j := uint8(0); j < pd.TriCount; j++ {
		t := tile.DetailTris[(pd.TriBase+uint(j))*4:]
		var v [3]Vector3
		for k := 0; k < 3; k++ {
			if t[k] < poly.VertCount {
				v[k] = tile.Verts[poly.Verts[t[k]]]
			} else {
				v[k] = tile.Verts[(pd.VertBase + uint(t[k]-poly.VertCount))]
			}
		}

		if h, ok := closestHeightPointTriangle(closest, v[0], v[1], v[2]); ok {
			closest[1] = h
			break
		}
	}

	return
}

// @}

func computeTileHash(x, y, mask int) int {
	const h1 = uint(0x8da6b343) // Large multiplicative constants;
	const h2 = uint(0xd8163841) // here arbitrarily chosen primes
	n := h1*uint(x) + h2*uint(y)
	return int(n & uint(mask))
}

func allocLink(tile *MeshTile) uint {
	if tile.LinksFreeList == DT_NULL_LINK {
		return DT_NULL_LINK
	}

	link := tile.LinksFreeList
	tile.LinksFreeList = tile.Links[link].next
	return link
}

func getSlabCoord(va Vector3, side int) float32 {
	if side == 0 || side == 4 {
		return va[0]
	} else if side == 2 || side == 6 {
		return va[2]
	}
	return 0
}

func overlapSlabs(amin, amax, bmin, bmax [2]float32, px, py float32) bool {
	// Check for horizontal overlap.
	// The segment is shrunken a little so that slabs which touch
	// at end points are not connected.
	minx := fmax(amin[0]+px, bmin[0]+px)
	maxx := fmin(amax[0]-px, bmax[0]-px)
	if minx > maxx {
		return false
	}

	// Check vertical overlap.
	ad := (amax[1] - amin[1]) / (amax[0] - amin[0])
	ak := amin[1] - ad*amin[0]
	bd := (bmax[1] - bmin[1]) / (bmax[0] - bmin[0])
	bk := bmin[1] - bd*bmin[0]
	aminy := ad*minx + ak
	amaxy := ad*maxx + ak
	bminy := bd*minx + bk
	bmaxy := bd*maxx + bk
	dmin := bminy - aminy
	dmax := bmaxy - amaxy

	// Crossing segments always overlap.
	if dmin*dmax < 0 {
		return true
	}

	// Check for overlap at endpoints.
	thr := sqr(py * 2)
	return dmin*dmin <= thr || dmax*dmax <= thr
}

func calcSlabEndPoints(va, vb Vector3, side int) (bmin, bmax [2]float32) {
	if side == 0 || side == 4 {
		if va[2] < vb[2] {
			bmin[0] = va[2]
			bmin[1] = va[1]
			bmax[0] = vb[2]
			bmax[1] = vb[1]
		} else {
			bmin[0] = vb[2]
			bmin[1] = vb[1]
			bmax[0] = va[2]
			bmax[1] = va[1]
		}
	} else if side == 2 || side == 6 {
		if va[0] < vb[0] {
			bmin[0] = va[0]
			bmin[1] = va[1]
			bmax[0] = vb[0]
			bmax[1] = vb[1]
		} else {
			bmin[0] = vb[0]
			bmin[1] = vb[1]
			bmax[0] = va[0]
			bmax[1] = va[1]
		}
	}

	return
}
