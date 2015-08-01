/*
* ***** BEGIN GPL LICENSE BLOCK *****
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software  Foundation,
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
* Contributor(s): Nicholas Bishop
*
* ***** END GPL LICENSE BLOCK *****
*
*/

/** \file blender/modifiers/intern/MOD_skin.c
*  \ingroup modifiers
*/

/* Implementation based in part off the paper "B-Mesh: A Fast Modeling
* System for Base Meshes of 3D Articulated Shapes" (Zhongping Ji,
* Ligang Liu, Yigang Wang)
*
* Note that to avoid confusion with Blender's BMesh data structure,
* this tool is renamed as the Skin modifier.
*
* The B-Mesh paper is current available here:
* http://www.math.zju.edu.cn/ligangliu/CAGD/Projects/BMesh/
*
* The main missing features in this code compared to the paper are:
*
* + No mesh evolution. The paper suggests iteratively subsurfing the
*   skin output and adapting the output to better conform with the
*   spheres of influence surrounding each vertex.
*
* + No mesh fairing. The paper suggests re-aligning output edges to
*   follow principal mesh curvatures.
*
* + No auxiliary balls. These would serve to influence mesh
*   evolution, which as noted above is not implemented.
*
* The code also adds some features not present in the paper:
*
* + Loops in the input edge graph.
*
* + Concave surfaces around branch nodes. The paper does not discuss
*   how to handle non-convex regions; this code adds a number of
*   cleanup operations to handle many (though not all) of these
*   cases.
*/

#include "MEM_guardedalloc.h"

#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_modifier_types.h"

#include "BLI_utildefines.h"
#include "BLI_array.h"
#include "BLI_heap.h"
#include "BLI_math.h"
#include "BLI_stack.h"
#include "BLI_bitmap.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_deform.h"
#include "BKE_DerivedMesh.h"
#include "BKE_mesh.h"
#include "BKE_mesh_mapping.h"
#include "BKE_modifier.h"

#include "bmesh.h"

static void skin_set_orig_indices(DerivedMesh *dm)
{
	int *orig, totpoly;

	totpoly = dm->getNumPolys(dm);
	orig = CustomData_add_layer(&dm->polyData, CD_ORIGINDEX,
		CD_CALLOC, NULL, totpoly);
	copy_vn_i(orig, totpoly, ORIGINDEX_NONE);
}

/**************************** Skin Modifier ***************************/
static void initData(ModifierData *md)
{
	BSkinModifierData *smd = (BSkinModifierData *)md;

	/* Enable in editmode by default */
	md->mode |= eModifierMode_Editmode;

	smd->branch_smoothing = 0;
	smd->flag = 0;
	smd->symmetry_axes = MOD_SKIN_SYMM_X;
}

static void copyData(ModifierData *md, ModifierData *target)
{
#if 0
	SkinModifierData *smd = (SkinModifierData *)md;
	SkinModifierData *tsmd = (SkinModifierData *)target;
#endif
	modifier_copyData_generic(md, target);
}

static DerivedMesh *applyModifier(ModifierData *md,
	Object *UNUSED(ob),
	DerivedMesh *dm,
	ModifierApplyFlag UNUSED(flag))
{
	return dm;
}

static CustomDataMask requiredDataMask(Object *UNUSED(ob),
	ModifierData *UNUSED(md))
{
	return CD_MASK_MVERT_SKIN | CD_MASK_MDEFORMVERT;
}

ModifierTypeInfo modifierType_BSkin = {
	/* name */              "BSkin",
	/* structName */        "BSkinModifierData",
	/* structSize */        sizeof(BSkinModifierData),
	/* type */              eModifierTypeType_Constructive,
	/* flags */             eModifierTypeFlag_AcceptsMesh | eModifierTypeFlag_SupportsEditmode,

	/* copyData */          copyData,
	/* deformVerts */       NULL,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     NULL,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     applyModifier,
	/* applyModifierEM */   NULL,
	/* initData */          initData,
	/* requiredDataMask */  requiredDataMask,
	/* freeData */          NULL,
	/* isDisabled */        NULL,
	/* updateDepgraph */    NULL,
	/* updateDepsgraph */   NULL,
	/* dependsOnTime */     NULL,
	/* dependsOnNormals */	NULL,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */     NULL,
};
//
//#include "MEM_guardedalloc.h"
//
//#include "DNA_meshdata_types.h"
//#include "DNA_object_types.h"
//#include "DNA_modifier_types.h"
//
//#include "BLI_utildefines.h"
//#include "BLI_array.h"
//#include "BLI_buffer.h"
//#include "BLI_alloca.h"
//#include "BLI_heap.h"
//#include "BLI_math.h"
//#include "BLI_stack.h"
//#include "BLI_bitmap.h"
//
//#include "BKE_cdderivedmesh.h"
//#include "BKE_deform.h"
//#include "BKE_DerivedMesh.h"
//#include "BKE_mesh.h"
//#include "BKE_mesh_mapping.h"
//#include "BKE_modifier.h"
//#include "bmesh.h"
//#include "BKE_global.h"
//
//#define STD_ALGORITHM_REVERSE_MACRO(REV_TYPE, first, last)  \
//		while ((first != last) && (first != --last)) {          \
//		SWAP(REV_TYPE, *first, *last); first++;            \
//			}											      
//
//enum
//{
//	/*SMM vertex*/
//	SQM_VERT_BRANCH = 1 << 0,
//	SQM_VERT_LEAF = 1 << 1,
//};
//enum
//{
//	/*BMesh edge*/
//	SKIN_BM_EDGE_MARK_SPLIT = 1 << 0,
//	SKIN_BM_EDGE_CONVEX = 1 << 1,
//	SKIN_BM_EDGE_TRIANGLE_MERGE = 1 << 2
//};
//enum
//{
//	SKIN_BM_FACE_VISITED = 1 << 0,
//	SKIN_BM_FACE_MARKED = 1 << 1,
//	SKIN_BM_FACE_ON_LIMB = 1 << 2,
//	SKIN_BM_FACE_ON_LEAF = 1 << 3,
//	SKIN_BM_FACE_ON_BRANCH = 1 << 4
//};
//enum
//{
//	SKIN_BM_VERT_DEGREE_0 = 1 << 0,
//	SKIN_BM_VERT_DEGREE_1 = 1 << 1,
//	SKIN_BM_VERT_DEGREE_2 = 1 << 2,
//	SKIN_BM_VERT_DEGREE_3 = 1 << 3,
//	SKIN_BM_VERT_ON_BRANCH = 1 << 4,
//	SKIN_BM_VERT_ON_LIMB = 1 << 5,
//	SKIN_BM_VERT_ON_LEAF = 1 << 6,
//	SKIN_BM_VERT_ON_CONNECTION = 1 << 7,
//	SKIN_BM_VERT_VISITED = 1 << 8
//};
//typedef struct
//{
//	int vert;
//	int flag;
//} SQMVert;
//
//typedef struct {
//	int v1;
//	int v2;
//	int*		verts;
//	int			tot;
//
//	/*path vertex on branch nodes*/
//	/*NULL when the corresponding nodes are leaf nodes*/
//	BMVert* path_v1;  /*corresponds to the v1 QDM node*/
//	BMVert* path_v2;  /*corresponds to the v2 QDM node*/
//} SQMEdge;
//
//typedef struct
//{
//	BMesh *bm;
//
//	int cd_vert_skin_offset;
//	int cd_vert_flag_offset; /*store QDM idx for branch vertex and (skin_v2 << 16| skin_v1) for limb edge*/
//	int cd_edge_flag_offset;
//	int cd_face_flag_offset;
//
//	SQMVert* sqm_verts;
//	SQMEdge* sqm_edges;
//
//	int		 tot_sqm_verts;
//	int		 tot_sqm_edges;
//	int*	 sqm_edge_vert_mem;
//
//	DerivedMesh* dm;
//	MVertSkin* sverts;
//	MVert*	verts;
//	MEdge*  edges;
//	int		totverts;
//	int		totedges;
//
//	MeshElemMap *emap;
//	int *emapmem;
//	MeshElemMap *sqm_emap;
//	int *sqm_emapmem;
//
//	BLI_Buffer* _cache_coord_quad_bridge0;
//	BLI_Buffer* _cache_coord_quad_bridge1;
//
//}SkinData;
//
//enum
//{
//	MVert_VISITED = 1 << 0
//};
//
//typedef float spoint3f[3];
//BMesh*   g_bmesh = NULL;
//BLI_buffer_declare(spoint3f, g_debug_points, BLI_BUFFER_NOP);
//BLI_buffer_declare(spoint3f, g_debug_segments, BLI_BUFFER_NOP);
//
//static void debug_init_data(SkinData* data)
//{
//	BLI_buffer_empty(&g_debug_points);
//	BLI_buffer_empty(&g_debug_segments);
//	if (g_bmesh)
//	{
//		BM_mesh_free(g_bmesh);
//		g_bmesh = BM_mesh_create(&bm_mesh_allocsize_default);
//		BM_mesh_elem_toolflags_ensure(g_bmesh);
//		BMO_push(g_bmesh, NULL);
//		bmesh_edit_begin(g_bmesh, 0);
//		data->bm = g_bmesh;
//	}
//	else
//	{
//		g_bmesh = BM_mesh_create(&bm_mesh_allocsize_default);
//		BM_mesh_elem_toolflags_ensure(g_bmesh);
//		BMO_push(g_bmesh, NULL);
//		bmesh_edit_begin(g_bmesh, 0);
//		data->bm = g_bmesh;
//	}
//}
//static void debug_draw_edges(BMesh* bmesh)
//{
//	if (bmesh)
//	{
//		BMEdge *e;
//		BMVert *ve = NULL;
//		BMIter iter, viter;
//
//		glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);
//		glLineWidth(0.6f);
//		//glDisable(GL_DEPTH_TEST);
//		glBegin(GL_LINES);
//		BM_ITER_MESH(e, &iter, bmesh, BM_EDGES_OF_MESH) {
//			BM_ITER_ELEM(ve, &viter, e, BM_VERTS_OF_EDGE)
//			{
//				glVertex3fv(ve->co);
//			}
//		}
//		glEnd();
//		//glEnable(GL_DEPTH_TEST);
//		glPopAttrib();
//	}
//}
//
//static void debug_draw_triangles(BMesh* bmesh)
//{
//	if (bmesh && bmesh->totface > 0)
//	{
//		BMFace *f;
//		BMVert *vf = NULL;
//		BMIter iter, viter;
//		BMVert* v[4];
//		float norm[3], centroid[3];
//
//		glPushAttrib(GL_CURRENT_BIT | GL_POLYGON_BIT | GL_LIGHTING_BIT);
//		glEnable(GL_LIGHTING);
//		glBegin(GL_TRIANGLES);
//		BM_ITER_MESH(f, &iter, bmesh, BM_FACES_OF_MESH) {
//			BM_face_normal_update(f);
//			if (f->len == 3)
//				BM_iter_as_array(NULL, BM_VERTS_OF_FACE, f, v, 3);
//			else if (f->len == 4)
//				BM_iter_as_array(NULL, BM_VERTS_OF_FACE, f, v, 4);
//			if (f->len == 3 || f->len == 4)
//			{
//				glNormal3fv(f->no);
//				glVertex3fv(v[0]->co);
//				glVertex3fv(v[1]->co);
//				glVertex3fv(v[2]->co);
//				if (f->len == 4){
//					glNormal3fv(f->no);
//					glVertex3fv(v[0]->co);
//					glVertex3fv(v[2]->co);
//					glVertex3fv(v[3]->co);
//				}
//			}
//		}
//		glEnd();
//
//		/*render normal*/
//		if (G.debug_value & 1 << 2){
//			glBegin(GL_LINES);
//			BM_ITER_MESH(f, &iter, bmesh, BM_FACES_OF_MESH) {
//				BM_face_calc_center_mean(f, centroid);
//				glVertex3fv(centroid);
//				add_v3_v3(centroid, f->no);
//				glVertex3fv(centroid);
//			}
//			glEnd();
//		}
//		glPopAttrib();
//	}
//}
//
//static void debug_draw_bmesh()
//{
//	if (G.debug_value & 1 << 1)
//		debug_draw_triangles(g_bmesh);
//
//	debug_draw_edges(g_bmesh);
//}
//static void debug_draw_points()
//{
//	size_t gotit = 1;
//	glPushAttrib(GL_CURRENT_BIT | GL_POINT_BIT);
//	glPointSize(10.0f);
//	//glDisable(GL_DEPTH_TEST);
//	glBegin(GL_POINTS);
//	for (int i = 0; i < g_debug_points.count; ++i){
//		glVertex3fv(BLI_buffer_at(&g_debug_points, spoint3f, i));
//	}
//	glEnd();
//	//glEnable(GL_DEPTH_TEST);
//	glPopAttrib();
//}
//
//static void debug_draw_segents()
//{
//	size_t gotit = 1;
//	glPushAttrib(GL_CURRENT_BIT | GL_POINT_BIT | GL_COLOR_BUFFER_BIT);
//	glLineWidth(3.0f);
//	glColor3f(1.0, 0.0, 0.0);
//	glBegin(GL_LINES);
//	for (int i = 0; i < g_debug_segments.count; ++i){
//		glVertex3fv(BLI_buffer_at(&g_debug_segments, spoint3f, i));
//	}
//	glEnd();
//	glPopAttrib();
//}
//static void debug_draw_kskin(void* data)
//{
//	debug_draw_bmesh();
//	debug_draw_points();
//	debug_draw_segents();
//}
//
//static void debug_segment_append(float p0[3], float p1[3])
//{
//	BLI_buffer_resize(&g_debug_segments, g_debug_segments.count + 2);
//	spoint3f* data0 = BLI_buffer_at(&g_debug_segments, spoint3f, g_debug_segments.count - 1);
//	spoint3f* data1 = BLI_buffer_at(&g_debug_segments, spoint3f, g_debug_segments.count - 2);
//	copy_v3_v3(data0, p0);
//	copy_v3_v3(data1, p1);
//}
//static void debug_point_coord_append(float coord[3])
//{
//	BLI_buffer_resize(&g_debug_points, g_debug_points.count + 1);
//	spoint3f* dpoint = BLI_buffer_at(&g_debug_points, spoint3f, g_debug_points.count - 1);
//	copy_v3_v3(dpoint, coord);
//}
//static void debug_point_append(BMVert* v)
//{
//	BLI_buffer_resize(&g_debug_points, g_debug_points.count + 1);
//	spoint3f* dpoint = BLI_buffer_at(&g_debug_points, spoint3f, g_debug_points.count - 1);
//	copy_v3_v3(dpoint, v->co);
//}
//static void debug_copy_point(spoint3f* source, int tot)
//{
//	if (tot >= g_debug_points.count)
//		BLI_buffer_resize(&g_debug_points, tot);
//
//	memcpy(BLI_buffer_array(&g_debug_points, spoint3f), source, tot*sizeof(spoint3f));
//}
//
//static void edges_from_tri(BMesh *bm, BMVert *v_tri[3], BMEdge *e_tri[3])
//{
//	e_tri[0] = BM_edge_create(bm, v_tri[0], v_tri[1], NULL, BM_CREATE_NO_DOUBLE);
//	e_tri[1] = BM_edge_create(bm, v_tri[1], v_tri[2], NULL, BM_CREATE_NO_DOUBLE);
//	e_tri[2] = BM_edge_create(bm, v_tri[2], v_tri[0], NULL, BM_CREATE_NO_DOUBLE);
//}
//
//static void calc_bary_center(BMVert** verts, int tot, float bary_center[3])
//{
//	float bary[3];
//	int cnt = 0;
//	zero_v3(bary);
//
//	for (int i = 0; i < tot; i++)
//		add_v3_v3(bary, verts[i]->co);
//
//	mul_v3_fl(bary, 1.0f / (float)tot);
//	copy_v3_v3(bary_center, bary);
//}
//
//static bool check_vert_flag(BMVert* v, int cd_vert_flag_off, int bit)
//{
//	return (BM_ELEM_CD_GET_INT(v, cd_vert_flag_off) & bit);
//}
//
//static void set_vert_flag(BMVert* v, int cd_vert_flag_off, int bit)
//{
//	int flag = (BM_ELEM_CD_GET_INT(v, cd_vert_flag_off));
//	flag |= bit;
//	BM_ELEM_CD_SET_INT(v, cd_vert_flag_off, flag);
//}
//
//static bool check_face_flag(BMFace* f, int cd_face_flag_off, int bit)
//{
//	return (BM_ELEM_CD_GET_INT(f, cd_face_flag_off) & bit);
//}
//
//static void set_face_flag(BMFace* f, int cd_face_flag_off, int bit)
//{
//	int flag = (BM_ELEM_CD_GET_INT(f, cd_face_flag_off));
//	flag |= bit;
//	BM_ELEM_CD_SET_INT(f, cd_face_flag_off, flag);
//}
//
///*project p onto cylinder's surface
//return true when p is inside cylinder and false otherwise
//*/
//static bool project_on_cylinder(float c0[3], float r0, float c1[3], float r1, const float p[3], float projected[3])
//{
//	float axis[3], dir[3], l, l_proj, l_dir;
//	bool inside;
//
//	sub_v3_v3v3(axis, c1, c0);
//	l = normalize_v3(axis);
//	sub_v3_v3v3(dir, p, c0);
//	l_proj = dot_v3v3(axis, dir);
//	l_dir = normalize_v3(dir);
//
//	if (l_proj < 0){
//		inside = l_dir < r0;
//		madd_v3_v3v3fl(projected, c0, dir, r0);
//	}
//	else if (l_proj > l){
//		sub_v3_v3v3(dir, p, c1);
//		inside = dot_v3v3(dir, dir) < r1*r1;
//
//		normalize_v3(dir);
//		madd_v3_v3v3fl(projected, c1, dir, r1);
//	}
//	else{
//		/*project p on cylinder body*/
//		float p_on_axis[3], l_sqr_orthor_sqr;
//		l_sqr_orthor_sqr = l_dir * l_dir - l_proj*l_proj;
//
//		inside = l_sqr_orthor_sqr < r0 * r0;
//
//		madd_v3_v3v3fl(p_on_axis, c0, axis, l_proj);
//		sub_v3_v3v3(dir, p, p_on_axis); normalize_v3(dir);
//		madd_v3_v3v3fl(projected, p_on_axis, dir, r0);
//	}
//
//	return inside;
//}
///*
//project p onto the surface of cone-sphere
//return true when p is inside conesphere and false otherwise
//for more detail, look at
//"Fast Distance Computation Between a Point and Cylinders, Cones, Line-Swept Spheres and Cone-Spheres".
//*/
//static bool project_on_cone_sphere_1(float c0[3], float r0, float c1[3], float r1, const float p[3], float projected[3])
//{
//	float u[3], ap[3], a[3], ra, b[3], rb;
//	float l, s, g, raa, rbb, ha, hb, px, py, pxy, mx, my, pmx, pmy;
//	bool inside;
//
//	copy_v3_v3(a, c0);
//	copy_v3_v3(b, c1);
//	ra = r0;
//	rb = r1;
//
//	/*ensure a is bigger sphere*/
//	if (ra < rb)
//	{
//		swap_v3_v3(a, b);
//		SWAP(float, ra, rb);
//	}
//	else if (fabsf(ra - rb) < FLT_EPSILON){
//		/*a cylinder here*/
//		return project_on_cylinder(c0, r0, c1, r1, p, projected);
//	}
//
//	sub_v3_v3v3(u, b, a);
//	l = normalize_v3(u);
//	g = ra - rb;
//
//	if (l < g){
//		/*b is inside a*/
//		copy_v3_v3(projected, p);
//		return false;
//	}
//
//	s = sqrtf(l*l - g*g);
//	ha = ra*g / l;
//	hb = rb*g / l;
//	raa = s*ra / l;
//	rbb = s*rb / l;
//
//	/*m coordinate*/
//	mx = ha;
//	my = raa;
//
//	/*p coordinate in a's reference*/
//	sub_v3_v3v3(ap, p, a);
//	px = dot_v3v3(ap, u);
//	pxy = normalize_v3(ap);
//	//madd_v3_v3v3fl(p1, a, u, px);
//	py = sqrtf(pxy*pxy - px*px);
//
//	/*p coordinate in m's reference*/
//	pmx = (px - mx) * s / l - (py - my) * g / l;
//	pmy = (px - mx) * g / l + (py - my) * s / l;
//
//	if (pmx < 0){
//		madd_v3_v3v3fl(projected, a, ap, ra);
//		/*inside*/
//		inside = pxy < ra;
//	}
//	else if (pmx > s){
//		float bp[3];
//		sub_v3_v3v3(bp, p, b);
//		inside = dot_v3v3(bp, bp) < rb * rb;
//		normalize_v3(bp);
//		madd_v3_v3v3fl(projected, b, bp, rb);
//	}
//	else {
//		inside = pmy < 0;
//
//		/*project on cylinder body*/
//		float p_on_axis[3], p_tilted[3], bb[3], dir[3];
//		float tilted_len, titled_len_x, l_b, r_projected;
//		madd_v3_v3v3fl(p_on_axis, a, u, px);
//		tilted_len = py * l / s;
//		titled_len_x = sqrtf(tilted_len * tilted_len - py*py);
//		madd_v3_v3v3fl(p_tilted, p_on_axis, u, -titled_len_x);
//		madd_v3_v3v3fl(bb, b, u, -hb);
//		l_b = len_v3v3(p_tilted, bb);
//		r_projected = l_b * g / l + rb;
//		sub_v3_v3v3(dir, p, p_tilted); normalize_v3(dir);
//		madd_v3_v3v3fl(projected, p_tilted, dir, r_projected);
//	}
//
//	return inside;
//}
//
///*for more detail, look at
//"Fast Distance Computation Between a Point and Cylinders, Cones, Line-Swept Spheres and Cone-Spheres"
//this function only project points INSIDE cone-sphere onto its surface.
//*/
//static bool project_on_cone_sphere_2(float c0[3], float r0, float c1[3], float r1, const float p[3], float projected[3])
//{
//	float u[3], ap[3], a[3], ra, b[3], rb;
//	float l, s, g, raa, rbb, ha, hb, px, py, pxy, mx, my, pmx, pmy;
//
//	copy_v3_v3(a, c0);
//	copy_v3_v3(b, c1);
//	ra = r0;
//	rb = r1;
//
//	/*ensure a is bigger sphere*/
//	if (ra < rb)
//	{
//		swap_v3_v3(a, b);
//		SWAP(float, ra, rb);
//	}
//	else if (fabsf(ra - rb) < FLT_EPSILON){
//		/*a cylinder here*/
//		return project_on_cylinder(c0, r0, c1, r1, p, projected);
//	}
//
//	sub_v3_v3v3(u, b, a);
//	l = normalize_v3(u);
//	g = ra - rb;
//	s = sqrtf(l*l - g*g);
//	ha = ra*g / l;
//	hb = rb*g / l;
//	raa = s*ra / l;
//	rbb = s*rb / l;
//
//	/*m coordinate*/
//	mx = ha;
//	my = raa;
//
//	/*p coordinate in a's reference*/
//	sub_v3_v3v3(ap, p, a);
//	px = dot_v3v3(ap, u);
//	pxy = normalize_v3(ap);
//	//madd_v3_v3v3fl(p1, a, u, px);
//	py = sqrtf(pxy*pxy - px*px);
//
//	/*p coordinate in m's reference*/
//	pmx = (px - mx) * s / l - (py - my) * g / l;
//	pmy = (px - mx) * g / l + (py - my)* s / l;
//
//	if (pmx < 0){
//		if (pxy < ra){
//			/*p is inside sphere a*/
//			madd_v3_v3v3fl(projected, a, ap, ra);
//			return true;
//		}
//	}
//	else if (pmx > s){
//		float bp[3];
//		sub_v3_v3v3(bp, p, b);
//		if (dot_v3v3(bp, bp) < rb * rb){
//			/*p is inside sphere b*/
//			normalize_v3(bp);
//			madd_v3_v3v3fl(projected, b, bp, rb);
//			return true;
//		}
//	}
//	else {
//		if (pmy < 0)
//		{
//			/*p is inside cylinder body*/
//			/*project on cylinder body*/
//			float p_on_axis[3], p_tilted[3], bb[3], dir[3];
//			float tilted_len, titled_len_x, l_b, r_projected;
//			madd_v3_v3v3fl(p_on_axis, a, u, px);
//			tilted_len = py * l / s;
//			titled_len_x = sqrtf(tilted_len * tilted_len - py*py);
//			madd_v3_v3v3fl(p_tilted, p_on_axis, u, -titled_len_x);
//			madd_v3_v3v3fl(bb, b, u, -hb);
//			l_b = len_v3v3(p_tilted, bb);
//			r_projected = l_b * g / l + rb;
//			sub_v3_v3v3(dir, p, p_tilted); normalize_v3(dir);
//			madd_v3_v3v3fl(projected, p_tilted, dir, r_projected);
//			return true;
//		}
//	}
//
//	copy_v3_v3(projected, p);
//	return false;
//}
//
//static void split_face(SkinData *data, BMFace* face, BMVert* nv)
//{
//	BMLoop* l;
//	BMVert* v[3];
//	BMEdge* e[3];
//	BMIter iter;
//	BMesh* bm = data->bm;
//	BM_ITER_ELEM(l, &iter, face, BM_LOOPS_OF_FACE){
//		v[0] = l->v;
//		v[1] = l->next->v;
//		v[2] = nv;
//		edges_from_tri(bm, v, e);
//		BM_face_create(bm, v, e, 3, face, BM_CREATE_NOP);
//	}
//	BM_face_kill(bm, face);
//}
//
///* Generates a map where the key is the vertex and the value is a list
//* of edges that use that vertex as an endpoint. The lists are allocated
//* from one memory pool. */
///*copy from BKE_mesh_vert_edge_map_create*/
//static void sqm_graph_vert_edge_map_create(MeshElemMap **r_map, int **r_mem,
//	const SQMEdge *medge, int totvert, int totedge)
//{
//	MeshElemMap *map = MEM_callocN(sizeof(MeshElemMap) * (size_t)totvert, "skin-vert-edge map");
//	int *indices = MEM_mallocN(sizeof(int[2]) * (size_t)totedge, "skin-vert-edge map mem");
//	int *i_pt = indices;
//
//	int i;
//
//	/* Count number of edges for each vertex */
//	for (i = 0; i < totedge; i++) {
//		map[medge[i].v1].count++;
//		map[medge[i].v2].count++;
//	}
//
//	/* Assign indices mem */
//	for (i = 0; i < totvert; i++) {
//		map[i].indices = i_pt;
//		i_pt += map[i].count;
//
//		/* Reset 'count' for use as index in last loop */
//		map[i].count = 0;
//	}
//
//	/* Find the users */
//	for (i = 0; i < totedge; i++) {
//		const unsigned int v[2] = { medge[i].v1, medge[i].v2 };
//
//		map[v[0]].indices[map[v[0]].count] = i;
//		map[v[1]].indices[map[v[1]].count] = i;
//
//		map[v[0]].count++;
//		map[v[1]].count++;
//	}
//
//	*r_map = map;
//	*r_mem = indices;
//}
//
///*
//- allocate sqm vertex array to SkinData.sqm_verts
//- allocate sqm edge array to SKinData.sqm_edges
//- build sqm graph from sqm vertex and sqm edges
//- build sqm edge vertex map to SkinDaa.sqm_emap
//*/
//static void build_sqm_graph(SkinData* dat)
//{
//#define MAX_STACK_DEPTH  500
//	BLI_Stack* stack;
//	SQMEdge* sqmcuredge = NULL;
//	int* prevvert;  /*reference to the previous vertex of the current vertex during stack traversal*/
//	int vcnt = 0, i = 0, offset = 0, vstart = -1;
//	int tmpverts[MAX_STACK_DEPTH];
//
//	MVert* verts = dat->verts;
//	MEdge* edges = dat->edges;
//	int totverts = dat->totverts;
//	int totedges = dat->totedges;
//	int tot_sqm_verts = 0;
//	MeshElemMap* emap = dat->emap;
//	GHash* sqm_vert_map;
//	/*initialze sqm vertex array*/
//	for (i = 0; i < totverts; ++i)
//	{
//		bool isBranchOrLeaf = emap[i].count > 2 || emap[i].count == 1;
//		if (isBranchOrLeaf)
//			tot_sqm_verts++;
//	}
//
//	dat->sqm_verts = MEM_callocN(sizeof(SQMVert)*tot_sqm_verts, "skin_sqmverts");
//	dat->sqm_edge_vert_mem = MEM_mallocN(sizeof(int) * 2 * dat->totverts, "skin_edge_vert");
//	dat->sqm_edges = MEM_callocN(sizeof(SQMEdge)*(size_t)dat->totedges, "skin_edge"); //allocate maximum number of sqm-edge
//	dat->tot_sqm_edges = 0;
//
//	sqm_vert_map = BLI_ghash_int_new("skin:sdmvertmap");
//	tot_sqm_verts = 0;
//	for (i = 0; i < totverts; ++i)
//	{
//		int num_adj = emap[i].count;
//		bool isBranchOrLeaf = num_adj > 2 || num_adj == 1;
//		if (isBranchOrLeaf)
//		{
//			dat->sqm_verts[tot_sqm_verts].vert = i;
//			if (num_adj > 2)
//				dat->sqm_verts[tot_sqm_verts].flag |= SQM_VERT_BRANCH;
//			else
//				dat->sqm_verts[tot_sqm_verts].flag |= SQM_VERT_LEAF;
//
//			BLI_ghash_insert(sqm_vert_map, i, tot_sqm_verts);
//			tot_sqm_verts++;
//			vstart = i;
//		}
//	}
//	dat->tot_sqm_verts = tot_sqm_verts;
//
//	/*initialize sqm graph*/
//	BLI_assert(vstart != -1);
//	prevvert = MEM_mallocN(sizeof(int)*(size_t)totverts, "skin_prevvert");
//	for (i = 0; i < totverts; i++) prevvert[i] = -1;
//
//	stack = BLI_stack_new(sizeof(int), "skin_stack");
//
//	BLI_bitmap* v_visited = BLI_BITMAP_NEW(totverts, "skin_v_visited");
//	BLI_BITMAP_ENABLE(v_visited, vstart);
//	bool test = BLI_BITMAP_TEST(v_visited, vstart);
//	for (i = 0; i < emap[vstart].count; ++i)
//	{
//		int vid = BKE_mesh_edge_other_vert(&edges[emap[vstart].indices[i]], vstart);
//		BLI_stack_push(stack, &vid);
//		prevvert[vid] = vstart;
//	}
//
//	while (!BLI_stack_is_empty(stack))
//	{
//		int vid = -1;
//		MeshElemMap* adj;
//
//		BLI_stack_pop(stack, &vid);
//		adj = &emap[vid];
//
//		BLI_BITMAP_ENABLE(v_visited, vid);
//
//		if (prevvert[vid] != -1)
//		{
//			/*if its previous node is a leaf or branch node ==> we have started a new chain
//			save its previous vertex into our vertex chain*/
//			int prevvalence = emap[prevvert[vid]].count;
//			bool prev_is_branch_or_leaf = prevvalence > 2 || prevvalence == 1;
//			if (prev_is_branch_or_leaf)
//			{
//				sqmcuredge = &dat->sqm_edges[dat->tot_sqm_edges++];
//				/*assign the first leaf or branch vertex of the chain*/
//				sqmcuredge->v1 = BLI_ghash_lookup(sqm_vert_map, prevvert[vid]);
//			}
//		}
//
//		bool is_branch_or_leaf = adj->count > 2 || adj->count == 1;
//		if (is_branch_or_leaf && sqmcuredge)
//		{
//			/*and a new chain had been stared earlier*/
//			/*assign the last leaf or branch vertex*/
//			sqmcuredge->v2 = BLI_ghash_lookup(sqm_vert_map, vid);
//			sqmcuredge->tot = vcnt;
//			sqmcuredge->verts = &dat->sqm_edge_vert_mem[offset];
//			offset += vcnt;
//			memcpy(sqmcuredge->verts, tmpverts, sizeof(int)*sqmcuredge->tot);
//			vcnt = 0;
//			sqmcuredge = NULL;
//		}
//		else
//		{
//			/*connection node*/
//			tmpverts[vcnt++] = vid;
//		}
//
//
//		/*keep going deep*/
//		for (i = 0; i < adj->count; ++i)
//		{
//			int adjv = BKE_mesh_edge_other_vert(&edges[adj->indices[i]], vid);
//			if (!BLI_BITMAP_TEST(v_visited, adjv))
//			{
//				BLI_stack_push(stack, &adjv);
//				prevvert[adjv] = vid;
//			}
//		}
//	}
//
//	/*initialize the edge map for sqm graph*/
//	sqm_graph_vert_edge_map_create(&dat->sqm_emap, &dat->sqm_emapmem, dat->sqm_edges, dat->tot_sqm_verts, dat->tot_sqm_edges);
//
//	BLI_ghash_free(sqm_vert_map, NULL, NULL);
//	BLI_stack_free(stack);
//	MEM_freeN(v_visited);
//	MEM_freeN(prevvert);
//
//#undef MAX_STACK_DEPTH
//}
//static float skin_radius(float* radius)
//{
//	return 0.5f*(radius[0] + radius[1]);
//}
///*
//return the nearest vertex from sqmv along the vertex chain of edge
//*/
//static int sqm_graph_nearest_vertex(SkinData* dat, const SQMEdge* edge, int sqmv)
//{
//	if (edge->tot > 0)
//	{
//		if (edge->v1 == sqmv)
//			return edge->verts[0];
//		else if (edge->v2 == sqmv)
//			return edge->verts[edge->tot - 1];
//		else
//			return -1;
//	}
//	else
//	{
//		int qdmvert = (edge->v1 == sqmv) ? edge->v2 : ((edge->v2 == sqmv) ? edge->v1 : -1);
//		if (qdmvert != -1)
//			return dat->sqm_verts[qdmvert].vert;
//		else
//			return -1;
//	}
//}
//static bool add_triangle_3_vertex_hull(SkinData* data, BMVert** inverts, int tot)
//{
//	BLI_assert(tot == 3);
//
//	int i, idx;
//	BMFace* face = NULL;
//	BMEdge* edges[3], *e;
//	BMVert* verts[3], *v;
//	BMLoop* loops[3];
//	BMIter iter;
//	BM_face_exists(inverts, tot, &face);
//	if (face)
//	{
//		BM_face_as_array_loop_tri(face, loops);
//		for (i = 0; i < 3; ++i)
//		{
//			edges[2 - i] = loops[i]->e;
//			verts[2 - i] = BM_edge_other_vert(loops[i]->e, loops[i]->v);
//		}
//
//		BMFace* nf = BM_face_create(data->bm, verts, edges, tot, face, BM_CREATE_NOP);
//
//		if (nf != NULL)
//			return true;
//		else
//			return false;
//	}
//	return false;
//}
//static bool build_hull(SkinData* data, BMVert** verts, int tot)
//{
//	int i;
//	BMOperator op;
//
//	BM_mesh_elem_hflag_disable_all(data->bm, BM_VERT, BM_ELEM_TAG, false);
//	for (i = 0; i < tot; ++i){
//		BM_elem_flag_enable(verts[i], BM_ELEM_TAG);
//	}
//
//	/* Deselect all faces so that only new hull output faces are
//	* selected after the operator is run */
//	BM_mesh_elem_hflag_disable_all(data->bm, BM_ALL_NOLOOP, BM_ELEM_SELECT, false);
//
//	BMO_op_initf(data->bm, &op, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
//		"convex_hull input=%hv", BM_ELEM_TAG);
//	BMO_op_exec(data->bm, &op);
//
//	if (BMO_error_occurred(data->bm)) {
//		BMO_op_finish(data->bm, &op);
//		return false;
//	}
//
//	BMO_op_finish(data->bm, &op);
//	BM_mesh_delete_hflag_tagged(data->bm, BM_ELEM_TAG, BM_EDGE | BM_FACE);
//
//	return true;
//}
//
//
//
//static void build_polyhedra(KSkinModifierData* smd, SkinData* data)
//{
//	int i, j;
//	int tot_sqm_verts = data->tot_sqm_verts;
//	int cd_vert_vskin_offset = data->cd_vert_skin_offset;
//
//
//	int cd_vert_flag_offset = data->cd_vert_flag_offset;
//
//	MeshElemMap* emap = data->emap;
//	MeshElemMap* sqm_emap = data->sqm_emap;
//	SQMVert*   sqm_verts = data->sqm_verts;
//	SQMEdge*   sqm_edges = data->sqm_edges;
//	MVertSkin* vertskins = data->sverts;
//	MVert*	   verts = data->verts, *v;
//	MEdge*	   edges = data->edges;
//	BMesh* bm = data->bm;
//
//	BMVert **vert_buf = NULL;
//	BLI_array_declare(vert_buf);
//
//	float size_inter = 20;
//	float inter0[3], inter1[3];
//
//	/*loop over branch node*/
//	for (i = 0; i < tot_sqm_verts; ++i)
//	{
//		if (sqm_verts[i].flag & SQM_VERT_BRANCH)
//		{
//			if (BLI_array_count(vert_buf) < sqm_emap[i].count) {
//				BLI_array_grow_items(vert_buf, (sqm_emap[i].count -
//					BLI_array_count(vert_buf)));
//			}
//
//			/*calc intersection between adjacent axises and the center sphere*/
//			float* center_coord = verts[sqm_verts[i].vert].co;
//			float  center_radius = skin_radius(vertskins[sqm_verts[i].vert].radius);
//			int    cnt_inter = 0;
//			for (j = 0; j < sqm_emap[i].count; ++j)
//			{
//				SQMEdge* sqmedge = &sqm_edges[sqm_emap[i].indices[j]];
//				int vadj = sqm_graph_nearest_vertex(data, sqmedge, i);
//				float* adj_coord = verts[vadj].co;
//
//				/*ignore this adjacent sphere if it is inside the center sphere*/
//				if (len_squared_v3v3(center_coord, adj_coord) < center_radius)
//					continue;
//
//				int ret = isect_line_sphere_v3(center_coord, adj_coord, center_coord, center_radius, inter0, inter1);
//				BLI_assert(ret == 1);
//
//				/*create vertex from projected point on sphere*/
//				vert_buf[cnt_inter] = BM_vert_create(bm, inter0, NULL, BM_CREATE_NO_DOUBLE);
//				BM_ELEM_CD_SET_INT(vert_buf[cnt_inter], cd_vert_vskin_offset, sqm_verts[i].vert);
//
//
//				set_vert_flag(vert_buf[cnt_inter], cd_vert_flag_offset, SKIN_BM_VERT_ON_BRANCH);
//
//				/*assign path vertices to SQM edge*/
//				if (sqmedge->v1 == i)
//					sqmedge->path_v1 = vert_buf[cnt_inter];
//				else
//					sqmedge->path_v2 = vert_buf[cnt_inter];
//
//				cnt_inter++;
//			}
//
//			if (cnt_inter >= 3){
//
//				/*build hull from vertices*/
//				build_hull(data, vert_buf, cnt_inter);
//
//				/*Hull algorithm create only one triangle for 3 vertices while we need two.
//				so, we add one more face in opposite winding*/
//				if (cnt_inter == 3){
//					add_triangle_3_vertex_hull(data, vert_buf, cnt_inter);
//				}
//			}
//		}
//	}
//
//	BLI_array_free(vert_buf);
//}
///*only project point inside sphere onto the sphere'surface*/
//static bool project_on_sphere(float center[3], float radius, float pin[3], float dir[3], float pout[3])
//{
//	float other[3], inter0[3], inter1[3];
//	int cnt_inter = 0;
//	mul_v3_v3fl(other, dir, 1000.0f);
//	add_v3_v3v3(other, pin, other);
//	cnt_inter = isect_line_sphere_v3(pin, other, center, radius, inter0, inter1);
//	if (cnt_inter == 1){
//		copy_v3_v3(pout, inter0);
//		return true;
//	}
//	else if (cnt_inter == 2){
//		sub_v3_v3v3(other, inter0, pin);
//		if (dot_v3v3(other, dir) >= 0.0f)
//			copy_v3_v3(pout, inter0);
//		else
//			copy_v3_v3(pout, inter1);
//		return true;
//	}
//	else{
//		return false;
//	}
//}
//static void calc_edge_sphere_projection_dir(BMesh* bm, BMEdge* edge, float dir[3])
//{
//	float mid[3], proj_dir[3], tmp[3];
//	BMFace* f;
//	BMIter iter;
//
//	/*used face normals as the projection dir*/
//	zero_v3(proj_dir);
//	BM_ITER_ELEM(f, &iter, edge, BM_FACES_OF_EDGE){
//		BM_face_calc_normal(f, tmp);
//		add_v3_v3(proj_dir, tmp);
//	}
//
//	/*if not, use adjacent centroid instead*/
//	if (len_squared_v3(proj_dir) < FLT_EPSILON)
//	{
//		zero_v3(proj_dir);
//		add_v3_v3v3(mid, edge->v1->co, edge->v2->co);
//		mul_v3_fl(mid, 0.5f);
//
//		BM_ITER_ELEM(f, &iter, edge, BM_FACES_OF_EDGE){
//			BM_face_calc_center_mean(f, tmp);
//			sub_v3_v3v3(tmp, mid, tmp);
//			normalize_v3(tmp);
//			add_v3_v3(proj_dir, tmp);
//		}
//	}
//
//	normalize_v3(proj_dir);
//	copy_v3_v3(dir, proj_dir);
//}
//
//static void find_plannar_region(SkinData *data, BMEdge* e, BLI_Buffer* faces, float epsilon)
//{
//	BMesh* bm;
//	BMFace* f;
//	BMEdge* ee;
//	BMIter iter;
//	int flag_f;
//	int cd_face_flag_off;
//	cd_face_flag_off = data->cd_face_flag_offset;
//
//	BLI_assert(BM_edge_is_manifold(e));
//
//	BLI_buffer_empty(faces);
//	BLI_Stack* stack = BLI_stack_new(sizeof(BMFace*), "kskin_p_region");
//
//	BLI_stack_push(stack, &e->l->f);
//	while (!BLI_stack_is_empty(stack)){
//
//		BLI_stack_pop(stack, &f);
//
//		flag_f = BM_ELEM_CD_GET_INT(f, cd_face_flag_off);
//		BM_ELEM_CD_SET_INT(f, cd_face_flag_off, flag_f | SKIN_BM_FACE_VISITED);
//
//		BLI_buffer_append(faces, BMFace*, f);
//
//		BM_ITER_ELEM(ee, &iter, f, BM_EDGES_OF_FACE){
//
//			BMFace* other_face = (ee->l->f == f) ? ee->l->radial_next->f : ee->l->f;
//
//			if (!(BM_ELEM_CD_GET_INT(other_face, cd_face_flag_off) & SKIN_BM_FACE_VISITED) &&
//				BM_edge_calc_face_angle(ee) < epsilon){
//				BLI_stack_push(stack, &other_face);
//			}
//		}
//	}
//}
//static void collect_verts_from_face_set(BLI_Buffer* faces, BLI_Buffer* verts)
//{
//	BMVert* v;
//	BMIter iter;
//	int i;
//
//	BLI_buffer_empty(verts);
//
//	for (i = 0; i < faces->count; ++i){
//
//		BMFace* f = BLI_buffer_at(faces, BMFace*, i);
//		BM_ITER_ELEM(v, &iter, f, BM_VERTS_OF_FACE){
//
//			if (!BM_elem_flag_test(v, BM_ELEM_TAG)){
//				BM_elem_flag_enable(v, BM_ELEM_TAG);
//				BLI_buffer_append(verts, BMVert*, v);
//			}
//		}
//	}
//
//	/*clear verts' flag*/
//	for (i = 0; i < verts->count; ++i){
//		BM_elem_flag_disable(BLI_buffer_at(verts, BMVert*, i), BM_ELEM_TAG);
//	}
//}
//
///*ensure the hole (boundary verts) running in opposite order to the winding order of remaining parts*/
///*this should be used when a new face created from such boundary verts*/
//static void ensure_winding_order_hole_boundary_verts(BMVert** verts, int tot)
//{
//	BMLoop *l_iter, *l_first;
//	BMVert *v1, *v0;
//	BMEdge* e;
//	BMFace* f;
//
//	/*two boundary verts*/
//	v0 = verts[0];
//	v1 = verts[1];
//
//	/*find the face containing two first verts*/
//	e = BM_edge_exists(v0, v1);
//	if (e){
//		f = e->l->f;
//		/*if adjacent exist, we need to make the hole run in opposite order to this adjacent face*/
//		if (f){
//			/*does two verts follow the winding order of face?*/
//			l_iter = l_first = f->l_first;
//			bool correct_winding = false;
//			do {
//				if (l_iter->v == v0)
//					correct_winding = (l_iter->next->v == v1);
//			} while ((l_iter = l_iter->next) != l_first);
//
//			/*if it is, we reverse the order of vertex array*/
//			if (correct_winding){
//				BMVert** first = &verts[0];
//				BMVert** last = first + tot;
//				STD_ALGORITHM_REVERSE_MACRO(BMVert*, first, last);
//			}
//		}
//	}
//}
//
//static bool sort_boundary_verts(BLI_Buffer* v_buffer, BLI_Buffer* faces, int cd_face_flag_off)
//{
//	int i, j, flag_f;
//	BMVert** verts;
//	BMFace* f0, *f1;
//	int tot_verts;
//	bool all_vert_on_boundary;
//
//	all_vert_on_boundary = true;
//	verts = v_buffer->data;
//	tot_verts = v_buffer->count;
//
//	/*mark the inner region*/
//	for (i = 0; i < faces->count; ++i){
//		BM_elem_flag_enable(BLI_buffer_at(faces, BMFace*, i), BM_ELEM_TAG);
//	}
//
//	for (i = 0; i < tot_verts - 1; ++i)
//	{
//		int next_v = -1;
//		/*find next vertex*/
//		for (j = i + 1; j < tot_verts; ++j)
//		{
//			BMEdge* e = BM_edge_exists(verts[i], verts[j]);
//			if (e && BM_edge_is_manifold(e)){
//				BM_edge_face_pair(e, &f0, &f1);
//				bool inner0 = BM_elem_flag_test(f0, BM_ELEM_TAG);
//				bool inner1 = BM_elem_flag_test(f1, BM_ELEM_TAG);
//				bool is_boundary_edge = (inner0 && !inner1) || (!inner0 && inner1);
//				if (is_boundary_edge){
//					next_v = j;
//					break;
//				}
//			}
//		}
//		if (next_v != -1){
//			SWAP(BMVert*, verts[i + 1], verts[next_v]);
//		}
//		else{
//			all_vert_on_boundary = false;
//			break;
//		}
//	}
//
//	/*un-mark the inner region*/
//	for (i = 0; i < faces->count; ++i){
//		BM_elem_flag_disable(BLI_buffer_at(faces, BMFace*, i), BM_ELEM_TAG);
//	}
//
//	return all_vert_on_boundary;
//
//}
///*in case where more than three vertex is connected on a convex hull, the topology is often bad
//so, we add a vertex to isolate them from each other*/
//static void isolate_path_verts(SkinData *data)
//{
//	BMIter iter, iter1;
//	BMEdge* e;
//	BMFace* f0, *f1, *new_face;
//	BMVert* new_vert;
//	int tot_edges, e_idx;
//	int cd_face_flag_off, cd_vert_flag_off, cd_vert_skin_off;
//	int i, skin_v;
//	float bary_center[3], avg_norm[3];
//	BMesh* bm;
//	MVert* mverts;
//	MVertSkin* sverts;
//	BLI_buffer_declare_static(BMFace*, faces, BLI_BUFFER_NOP, 20);
//	BLI_buffer_declare_static(BMVert*, verts, BLI_BUFFER_NOP, 20);
//
//	const double epsilon = M_PI / 6;
//
//	mverts = data->verts;
//	sverts = data->sverts;
//	cd_vert_skin_off = data->cd_vert_skin_offset;
//	cd_face_flag_off = data->cd_face_flag_offset;
//	cd_vert_flag_off = data->cd_vert_flag_offset;
//	bm = data->bm;
//	tot_edges = bm->totedge;
//
//	BM_mesh_normals_update(bm);
//	BM_mesh_elem_hflag_disable_all(bm, BM_FACE | BM_VERT, BM_ELEM_TAG, false);
//
//	BM_ITER_MESH_INDEX(e, &iter, bm, BM_EDGES_OF_MESH, e_idx)
//	{
//		if (e_idx >= tot_edges) break;
//		if (!BM_edge_is_manifold(e)) continue;
//
//		f0 = e->l->f;
//		f1 = e->l->radial_next->f;
//		BLI_assert(f0 != f1);
//		int flag_f0 = BM_ELEM_CD_GET_INT(f0, cd_face_flag_off);
//		int flag_f1 = BM_ELEM_CD_GET_INT(f1, cd_face_flag_off);
//		if (!(flag_f0 & SKIN_BM_FACE_VISITED) &&
//			!(flag_f1 & SKIN_BM_FACE_VISITED) &&
//			BM_edge_calc_face_angle(e) < epsilon) {
//
//			find_plannar_region(data, e, &faces, epsilon);
//
//			collect_verts_from_face_set(&faces, &verts);
//
//			if (verts.count > 3 && sort_boundary_verts(&verts, &faces, cd_face_flag_off))
//			{
//				/*kill old faces and calculate average normal*/
//				zero_v3(avg_norm);
//				for (i = 0; i < faces.count; ++i){
//					f0 = BLI_buffer_at(&faces, BMFace*, i);
//					add_v3_v3(avg_norm, f0->no);
//					BM_face_kill(bm, f0);
//				}
//				mul_v3_fl(avg_norm, 1.0f / faces.count);
//
//				/*ensure winding order before creating new face*/
//				ensure_winding_order_hole_boundary_verts(verts.data, verts.count);
//
//				/*create a new face*/
//				new_face = BM_face_create_verts(bm, verts.data, verts.count, NULL, BM_CREATE_NOP, false);
//				BLI_assert(new_face);
//
//				/*create a new vertex at bary center*/
//				calc_bary_center(verts.data, verts.count, bary_center);
//				skin_v = BM_ELEM_CD_GET_INT(BLI_buffer_at(&verts, BMVert*, 0), cd_vert_skin_off);
//				project_on_sphere(mverts[skin_v].co, skin_radius(sverts[skin_v].radius), bary_center, avg_norm, bary_center);
//				new_vert = BM_vert_create(bm, bary_center, BLI_buffer_at(&verts, BMVert*, 0), BM_CREATE_NO_DOUBLE);
//
//				/*split new_f*/
//				split_face(data, new_face, new_vert);
//
//				/*mark new split faces as VISITED*/
//				BM_ITER_ELEM(f0, &iter1, new_vert, BM_FACES_OF_VERT){
//					BM_face_normal_update(f0);
//					int flag = BM_ELEM_CD_GET_INT(f0, cd_face_flag_off);
//					BM_ELEM_CD_SET_INT(f0, cd_face_flag_off, flag | SKIN_BM_FACE_VISITED);
//				}
//			}
//		}
//	}
//
//	BM_ITER_MESH_INDEX(e, &iter, bm, BM_EDGES_OF_MESH, e_idx)
//	{
//		if (BM_edge_is_wire(e))
//			BM_edge_kill(bm, e);
//	}
//}
//
//static void split_edges(SkinData *data)
//{
//	MVert* verts;
//	MVertSkin* vskins;
//	BMEdge *e, *e_r;
//	BMFace *f;
//	BMIter iter, fiter;
//	float mid[3], proj_dir[3], tmp[3], scenter[3], radius;
//	int vid, eindex, totedge;
//	BMesh *bm;
//
//	bm = data->bm;
//	verts = data->verts;
//	vskins = data->sverts;
//	totedge = data->bm->totedge;
//
//	BM_mesh_elem_hflag_disable_all(bm, BM_EDGE, BM_ELEM_TAG, false);
//
//	BM_ITER_MESH(e, &iter, bm, BM_EDGES_OF_MESH){
//
//		if (BM_elem_flag_test(e, BM_ELEM_TAG)) continue;
//
//		calc_edge_sphere_projection_dir(bm, e, proj_dir);
//
//		BMVert *v = BM_edge_split(data->bm, e, e->v1, &e_r, 0.5f);
//		copy_v3_v3(mid, v->co);
//
//		BM_elem_flag_enable(e, BM_ELEM_TAG);
//		BM_elem_flag_enable(e_r, BM_ELEM_TAG);
//
//		vid = BM_ELEM_CD_GET_INT(e->v1, data->cd_vert_skin_offset);
//		copy_v3_v3(scenter, verts[vid].co);
//		radius = skin_radius(vskins[vid].radius);
//
//		if (project_on_sphere(scenter, radius, mid, proj_dir, tmp))
//			copy_v3_v3(v->co, tmp);
//		else
//			BLI_assert(false);
//	}
//}
//
//static void bm_edges_from_quad(BMesh *bm, BMVert *v_quad[4], BMEdge *e_quad[4])
//{
//	e_quad[0] = BM_edge_create(bm, v_quad[0], v_quad[1], NULL, BM_CREATE_NO_DOUBLE);
//	e_quad[1] = BM_edge_create(bm, v_quad[1], v_quad[2], NULL, BM_CREATE_NO_DOUBLE);
//	e_quad[2] = BM_edge_create(bm, v_quad[2], v_quad[3], NULL, BM_CREATE_NO_DOUBLE);
//	e_quad[3] = BM_edge_create(bm, v_quad[3], v_quad[0], NULL, BM_CREATE_NO_DOUBLE);
//}
//
//static void bm_edges_from_tri(BMesh *bm, BMVert *v_tri[3], BMEdge *e_tri[3])
//{
//	e_tri[0] = BM_edge_create(bm, v_tri[0], v_tri[1], NULL, BM_CREATE_NO_DOUBLE);
//	e_tri[1] = BM_edge_create(bm, v_tri[1], v_tri[2], NULL, BM_CREATE_NO_DOUBLE);
//	e_tri[2] = BM_edge_create(bm, v_tri[2], v_tri[0], NULL, BM_CREATE_NO_DOUBLE);
//}
//
//
//static void split_faces(SkinData *data)
//{
//	BMVert* v = NULL, *nv = NULL;
//	BMFace* f, *ff;
//	BMIter faceiter, faceiter1, vertiter;
//	MVert* verts;
//	MVertSkin* sverts;
//	int totface, findex, vid;
//	BMesh* bm;
//	float centroid[3], sphere_center[3], radius, fnormal[3], tmp[3];
//
//	bm = data->bm;
//	verts = data->verts;
//	sverts = data->sverts;
//	totface = bm->totface;
//
//	BM_mesh_elem_hflag_disable_all(bm, BM_FACE, BM_ELEM_TAG, false);
//
//	BM_ITER_MESH(f, &faceiter, bm, BM_FACES_OF_MESH){
//
//		if (BM_elem_flag_test(f, BM_ELEM_TAG)) continue;
//
//		BM_face_calc_center_mean(f, centroid);
//		BM_face_calc_normal(f, fnormal);
//
//		v = BM_FACE_FIRST_LOOP(f)->v;
//
//		vid = BM_ELEM_CD_GET_INT(v, data->cd_vert_skin_offset);
//		copy_v3_v3(sphere_center, verts[vid].co);
//		radius = skin_radius(sverts[vid].radius);
//
//		if (project_on_sphere(sphere_center, radius, centroid, fnormal, tmp))
//			copy_v3_v3(centroid, tmp);
//		else
//			BLI_assert(false);
//
//		nv = BM_vert_create(bm, centroid, v, BM_CREATE_NOP);
//
//		split_face(data, f, nv);
//
//		/*tag all new faces*/
//		BM_ITER_ELEM(ff, &faceiter1, nv, BM_FACES_OF_VERT){
//			BM_elem_flag_enable(ff, BM_ELEM_TAG);
//		}
//	}
//}
//
//static void split(SkinData *data)
//{
//	split_edges(data);
//	split_faces(data);
//}
//
//static BMVert* split_edge(SkinData* data, BMEdge* edge)
//{
//	int i;
//	float mid[3];
//	BMLoop* edge_loop[2];
//	BMesh* bm = data->bm;
//	BMVert* v_new;
//
//	if (BM_edge_loop_pair(edge, &edge_loop[0], &edge_loop[1]))
//	{
//		/* Create a new vertex in current node at the edge's midpoint */
//		mid_v3_v3v3(mid, edge->v1->co, edge->v2->co);
//		v_new = BM_vert_create(bm, mid, edge->v1, BM_CREATE_NOP);
//
//		for (i = 0; i < 2; ++i){
//			BMLoop *l_adj = edge_loop[i];
//			BMFace *f_adj = l_adj->f;
//			BMFace *f_new;
//			BMVert *v_opp, *v1, *v2;
//			BMVert *v_tri[3];
//			BMEdge *e_tri[3];
//
//			/* Find the vertex not in the edge */
//			v_opp = l_adj->prev->v;
//
//			/* Get e->v1 and e->v2 in the order they appear in the
//			* existing face so that the new faces' winding orders
//			* match */
//			v1 = l_adj->v;
//			v2 = l_adj->next->v;
//
//			/* Create two new faces */
//			v_tri[0] = v1;
//			v_tri[1] = v_new;
//			v_tri[2] = v_opp;
//			edges_from_tri(bm, v_tri, e_tri);
//			BMFace* face_new = BM_face_create(bm, v_tri, e_tri, 3, f_adj, BM_CREATE_NOP);
//
//			v_tri[0] = v_new;
//			v_tri[1] = v2;
//			/* v_tri[2] = v_opp; */ /* unchanged */
//			e_tri[0] = BM_edge_create(bm, v_tri[0], v_tri[1], NULL, BM_CREATE_NO_DOUBLE);
//			e_tri[2] = e_tri[1];  /* switched */
//			e_tri[1] = BM_edge_create(bm, v_tri[1], v_tri[2], NULL, BM_CREATE_NO_DOUBLE);
//			face_new = BM_face_create(bm, v_tri, e_tri, 3, f_adj, BM_CREATE_NOP);
//
//			BM_face_kill(bm, f_adj);
//		}
//
//		BM_edge_kill(bm, edge);
//		return v_new;
//	}
//
//	return NULL;
//}
//static bool increase_valence_path_vertex(SkinData* data, BMVert* v, int delta)
//{
//	BMesh* bm = data->bm;
//	BMIter iter, eiter;
//	BMFace* f;
//	BMEdge* e, *opp_e, *longest_edge = NULL;
//	float max_sqr_dst = FLT_MIN, sqr_dst, center[3], radius, proj_dir[3], tmp[3];
//	int i, v_idx;
//
//	v_idx = BM_ELEM_CD_GET_INT(v, data->cd_vert_skin_offset);
//	copy_v3_v3(center, data->verts[v_idx].co);
//	radius = skin_radius(data->sverts[v_idx].radius);
//
//	for (i = 0; i < delta; ++i){
//
//		/*find the longest edge around v*/
//		max_sqr_dst = FLT_MIN;
//		BM_ITER_ELEM(f, &iter, v, BM_FACES_OF_VERT){
//
//			/*find the opposite edge to v*/
//			BM_ITER_ELEM(e, &eiter, f, BM_EDGES_OF_FACE){
//				if (e->v1 != v && e->v2 != v){
//					opp_e = e;
//					break;
//				}
//			}
//
//			sqr_dst = BM_edge_calc_length_squared(opp_e);
//			if (sqr_dst > max_sqr_dst){
//				max_sqr_dst = sqr_dst;
//				longest_edge = opp_e;
//			}
//		}
//
//		/*split the longest edge*/
//		if (longest_edge){
//			calc_edge_sphere_projection_dir(bm, longest_edge, proj_dir);
//			BMVert* new_v = split_edge(data, longest_edge);
//			if (project_on_sphere(center, radius, new_v->co, proj_dir, tmp)){
//				copy_v3_v3(new_v->co, tmp);
//			}
//		}
//		else{
//			return false;
//		}
//	}
//	return true;
//}
//static void refine_valence(SkinData* data)
//{
//#define MIN_VALENCE 8
//	int i;
//	int valence;
//	int tot_sqm_edges = data->tot_sqm_edges;
//	SQMEdge* sqm_edges = data->sqm_edges, *max_dif_edge;
//
//	for (i = 0; i < tot_sqm_edges; ++i){
//		SQMEdge* sqme = &sqm_edges[i];
//		if (sqme->path_v1){
//			valence = BM_vert_edge_count(sqme->path_v1);
//			if (valence > 0 && valence < MIN_VALENCE){
//				increase_valence_path_vertex(data, sqme->path_v1, MIN_VALENCE - valence);
//				BLI_assert(BM_vert_edge_count(sqme->path_v1) == 8);
//			}
//		}
//		if (sqme->path_v2){
//			valence = BM_vert_edge_count(sqme->path_v2);
//			if (valence > 0 && valence < MIN_VALENCE){
//				increase_valence_path_vertex(data, sqme->path_v2, MIN_VALENCE - valence);
//				BLI_assert(BM_vert_edge_count(sqme->path_v2) == 8);
//			}
//		}
//	}
//#undef  MIN_VALENCE
//}
//static void equalize_valence(SkinData* data)
//{
//	int i, max_dif;
//	int valence1, valence2, val_dif;
//	SQMEdge* sqm_edges = data->sqm_edges, *max_dif_edge;
//	BMVert* v1, *v2;
//	int tot_sqm_edges = data->tot_sqm_edges;
//	bool done = false;
//
//	refine_valence(data);
//
//	while (true){
//
//		/*find the sqm edge with maximum valence difference*/
//		max_dif_edge = NULL;
//		max_dif = 0;
//		for (i = 0; i < tot_sqm_edges; ++i){
//			SQMEdge* sqme = &sqm_edges[i];
//			if (sqme->path_v1 && sqme->path_v2){
//				valence1 = BM_vert_edge_count(sqme->path_v1);
//				valence2 = BM_vert_edge_count(sqme->path_v2);
//				int val_dif = abs(valence2 - valence1);
//				if (valence1 > 0 && valence2 > 0 && val_dif > max_dif){
//					max_dif = val_dif;
//					max_dif_edge = sqme;
//				}
//			}
//		}
//
//		/*equalize valence*/
//		if (max_dif_edge){
//			v1 = max_dif_edge->path_v1;
//			v2 = max_dif_edge->path_v2;
//			valence1 = BM_vert_edge_count(v1);
//			valence2 = BM_vert_edge_count(v2);
//			val_dif = abs(valence1 - valence2);
//			if (val_dif > 0)
//			{
//				if (valence1 > valence2)
//					increase_valence_path_vertex(data, v2, val_dif);
//				else
//					increase_valence_path_vertex(data, v1, val_dif);
//			}
//		}
//		else
//			break;
//	}
//
//}
//
//static void generate_circle_verts(SkinData* data,
//	const float to_center_dir[3], const float center[3], float radius, int count,
//	BMVert** verts)
//{
//	float basis[2][3], coord[3];
//	float step, angle;
//	int i;
//	BMesh* bm = data->bm;
//
//	ortho_basis_v3v3_v3(basis[0], basis[1], to_center_dir);
//
//	step = 2.0f*M_PI / count;
//	for (i = 0; i < count; ++i){
//		angle = i*step;
//		mul_v3_v3fl(coord, basis[0], cosf(angle));
//		madd_v3_v3fl(coord, basis[1], sinf(angle));
//		normalize_v3(coord);
//		mul_v3_fl(coord, radius);
//		add_v3_v3v3(coord, center, coord);
//		verts[i] = BM_vert_create(bm, coord, NULL, BM_CREATE_NOP);
//	}
//}
//static void encode_cone_sphere(SkinData* data, BMVert* v, int skin_v1, int skin_v2)
//{
//	int cd_vert_skin_off = data->cd_vert_skin_offset;
//
//	BLI_assert(skin_v1 < (1 << 16 - 1) && skin_v1 < (1 << 16 - 1));
//
//	skin_v2 <<= 16;
//
//	int cone_sphere = skin_v1 | skin_v2;
//
//	BM_ELEM_CD_SET_INT(v, cd_vert_skin_off, cone_sphere);
//}
//
//static void decode_cone_sphere(SkinData* data, BMVert* v, int* skin_v1, int* skin_v2)
//{
//#ifdef _DEBUG
//	int flag = BM_ELEM_CD_GET_INT(v, data->cd_vert_flag_offset);
//	BLI_assert(flag & SKIN_BM_VERT_ON_LIMB);
//#endif
//
//	int cd_vert_skin_off = data->cd_vert_skin_offset;
//	int cone_sphere = BM_ELEM_CD_GET_INT(v, cd_vert_skin_off);
//	*skin_v1 = (cone_sphere & 0x0000FFFF);
//	*skin_v2 = (cone_sphere & 0xFFFF0000) >> 16;
//}
//
//static void set_verts_set_flag(BMVert** verts, int tot, int cd_vert_flag_off, int flag)
//{
//	int i;
//	for (i = 0; i < tot; ++i){
//		set_vert_flag(verts[i], cd_vert_flag_off, flag);
//	}
//}
//
//static void remove_adjacent_faces(SkinData* data, BMVert* v, BMVert** adj_verts, int valence)
//{
//	BMFace* f;
//	BMEdge* e_prev;
//	BMIter iter;
//	BMLoop *l_first;
//	BMLoop *l_iter;
//	BMesh* bm = data->bm;
//	int i;
//
//	/*take the first loop of v*/
//	BM_ITER_ELEM(l_first, &iter, v, BM_LOOPS_OF_VERT){
//		break;
//	}
//
//	i = 0;
//	l_iter = l_first;
//	e_prev = l_first->e;
//	adj_verts[i++] = BM_edge_other_vert(e_prev, v);
//	while (((l_iter = BM_vert_step_fan_loop(l_iter, &e_prev)) != l_first) && (l_iter != NULL)) {
//		adj_verts[i++] = BM_edge_other_vert(e_prev, v);
//	}
//
//	BM_vert_kill(bm, v);
//}
//
//
//static void std_algorithm_reverse(int* first, int* last)
//{
//	while ((first != last) && (first != --last)) {
//		SWAP(int, *first, *last); first++;
//	}
//}
//static void std_algorithm_rotate(int* first, int* n_first, int* last)
//{
//	int* next = n_first;
//	while (first != next) {
//		SWAP(int, *first, *next);
//		first++; next++;
//		if (next == last) {
//			next = n_first;
//		}
//		else if (first == n_first) {
//			n_first = next;
//		}
//	}
//}
//
//static int vert_skin_idx(BMVert* v, int off)
//{
//	return BM_ELEM_CD_GET_INT(v, off);
//}
//
//static void vert_skin_coord(BMVert* v, MVert* mverts, int cd_off, float coord[3])
//{
//	int idx = BM_ELEM_CD_GET_INT(v, cd_off);
//	copy_v3_v3(coord, mverts[idx].co);
//}
//
//static void skin_vert_coord_radius_from_v(BMVert* v, MVert* mverts, MVertSkin* sverts, int cd_off, float coord[3], float* radius)
//{
//	int idx = BM_ELEM_CD_GET_INT(v, cd_off);
//	copy_v3_v3(coord, mverts[idx].co);
//	*radius = skin_radius(sverts[idx].radius);
//}
//
//static void skin_vert_coord_radius_from_id(int skin_v, MVert* mverts, MVertSkin* sverts, float coord[3], float* radius)
//{
//	copy_v3_v3(coord, mverts[skin_v].co);
//	*radius = skin_radius(sverts[skin_v].radius);
//}
//
//static void calc_normal_along_dir(BMVert** vset, int tot, const float dir[3], float norm[3])
//{
//	float dir0[3], dir1[3];
//	sub_v3_v3v3(dir0, vset[1]->co, vset[0]->co);
//	sub_v3_v3v3(dir1, vset[2]->co, vset[1]->co);
//	cross_v3_v3v3(norm, dir0, dir1); normalize_v3(norm);
//	if (dot_v3v3(dir, norm) < 0.0f)
//		negate_v3_v3(norm, norm);
//}
//static void choose_bridge_order(SkinData* data, BMesh* bm, BMVert** vset0, BMVert** vset1, int tot, int* order)
//{
//	float min_sum, square_dst_sum;
//	int i, j;
//	float dir[3], norm0[3], norm1[3], bary0[3], bary1[3];
//	int* tmp_order = BLI_array_alloca(tmp_order, tot);
//
//	BLI_buffer_resize(data->_cache_coord_quad_bridge0, tot);
//	BLI_buffer_resize(data->_cache_coord_quad_bridge1, tot);
//
//	float(*coord_set0)[3] = (float*)data->_cache_coord_quad_bridge0->data;
//	float(*coord_set1)[3] = (float*)data->_cache_coord_quad_bridge1->data;
//
//	zero_v3(bary0); zero_v3(bary1);
//	for (i = 0; i < tot; ++i)
//	{
//		copy_v3_v3(coord_set0[i], vset0[i]->co);
//		copy_v3_v3(coord_set1[i], vset1[i]->co);
//		add_v3_v3(bary0, coord_set0[i]);
//		add_v3_v3(bary1, coord_set1[i]);
//	}
//	mul_v3_fl(bary0, 1.0f / (float)tot);
//	mul_v3_fl(bary1, 1.0f / (float)tot);
//
//	sub_v3_v3v3(dir, bary1, bary0); normalize_v3(dir);
//	calc_normal_along_dir(vset0, tot, dir, norm0);
//	calc_normal_along_dir(vset1, tot, dir, norm1);
//
//	/*the metrics "total sum of distances is not correct when the angle between two normal too large"*/
//	/*so, we project both vertex on two parallel planes*/
//	if (dot_v3v3(norm0, norm1) < 0.2f){
//		add_v3_v3v3(norm0, norm0, norm1); normalize_v3(norm0);
//		for (i = 0; i < tot; i++){
//			project_v3_plane(coord_set0[i], norm0, bary0);
//			project_v3_plane(coord_set1[i], norm0, bary1);
//		}
//	}
//
//	for (i = 0; i < tot; ++i)
//		tmp_order[i] = i;
//
//	min_sum = FLT_MAX;
//	for (i = 0; i < tot; ++i){
//
//		square_dst_sum = 0.0f;
//		for (j = 0; j < tot; ++j){
//			float square_dst = len_squared_v3v3(coord_set0[j], coord_set1[tmp_order[j]]);
//			square_dst_sum += square_dst;
//		}
//
//		if (square_dst_sum < min_sum){
//			min_sum = square_dst_sum;
//			memcpy(order, tmp_order, sizeof(int)*tot);
//		}
//
//		std_algorithm_rotate(tmp_order, tmp_order + 1, tmp_order + tot);
//	}
//
//	/*reversed order: tot-1, tot-2, ..., */
//	for (i = 0; i < tot; ++i)
//		tmp_order[tot - 1 - i] = i;
//
//	for (i = 0; i < tot; ++i){
//
//		square_dst_sum = 0.0f;
//		for (j = 0; j < tot; ++j){
//			float square_dst = len_squared_v3v3(coord_set0[j], coord_set1[tmp_order[j]]);
//			square_dst_sum += square_dst;
//		}
//
//		if (square_dst_sum < min_sum){
//			min_sum = square_dst_sum;
//			memcpy(order, tmp_order, sizeof(int)*tot);
//		}
//
//		std_algorithm_rotate(tmp_order, tmp_order + 1, tmp_order + tot);
//	}
//}
//
//static void bridge_vert_set(SkinData* data, BMesh* bm, BMVert** vset0, BMVert** vset1, int* order, int valence)
//{
//	int i, inext, fflag, cd_face_flag_off;
//	BMEdge* split_edge;
//	BMFace* f;
//	cd_face_flag_off = data->cd_face_flag_offset;
//
//	for (i = 0; i < valence; ++i){
//		inext = (i + 1) % valence;
//		f = BM_face_create_quad_tri(bm,
//			vset0[i],
//			vset0[inext],
//			vset1[order[inext]],
//			vset1[order[i]],
//			NULL, BM_CREATE_NOP);
//		/*mark new face as on limb*/
//		if (f){
//			set_face_flag(f, cd_face_flag_off, SKIN_BM_FACE_ON_LIMB);
//		}
//	}
//
//	/*mark one one bridge for splitting later*/
//	split_edge = BM_edge_exists(vset0[0], vset1[order[0]]);
//	BLI_assert(split_edge != NULL);
//	if (split_edge){
//		int a = BM_ELEM_CD_GET_INT(split_edge, data->cd_edge_flag_offset);
//		a |= SKIN_BM_EDGE_MARK_SPLIT;
//		BM_ELEM_CD_SET_INT(split_edge, data->cd_edge_flag_offset, a);
//	}
//}
//
///*calc two boundary segment of cone-sphere on the plane*/
//static void calc_cone_sphere_segment(
//	const float center0[3], float radius0,
//	const float center1[3], float radius1,
//	const float norm[3],
//	float segment0[2][3], float segment1[2][3])
//{
//	/*cone-sphere*/
//	if ((radius1 - radius0)*(radius1 - radius0) > FLT_EPSILON)
//	{
//		float a[3], b[3], ab[3], dir[3];
//		float ra, rb, l, gamma, ha, hb, hhb;
//
//		/*a is a bigger sphere*/
//		ra = (radius0 > radius1) ? radius0 : radius1;
//		rb = (radius0 > radius1) ? radius1 : radius0;
//		(radius0 > radius1) ? copy_v3_v3(a, center0) : copy_v3_v3(a, center1);
//		(radius0 > radius1) ? copy_v3_v3(b, center1) : copy_v3_v3(b, center0);
//
//		sub_v3_v3v3(ab, b, a);
//		l = normalize_v3(ab);
//		gamma = ra - rb;
//		ha = (ra *gamma) / l;
//		hb = (rb *gamma) / l;
//
//		float tmp = ra* ra - ha *ha;
//		float hha = sqrtf(tmp);
//		tmp = rb* rb - hb *hb;
//		hhb = sqrtf(tmp);
//		madd_v3_v3fl(a, ab, ha);
//		madd_v3_v3fl(b, ab, hb);
//		cross_v3_v3v3(dir, ab, norm); normalize_v3(dir);
//
//		madd_v3_v3v3fl(segment0[0], a, dir, hha);
//		madd_v3_v3v3fl(segment0[1], b, dir, hhb);
//
//		madd_v3_v3v3fl(segment1[0], a, dir, -hha);
//		madd_v3_v3v3fl(segment1[1], b, dir, -hhb);
//
//	}
//	else
//	{
//		/*cylinder*/
//		float axis[3], side_dir[3];
//		sub_v3_v3v3(axis, center0, center1);
//		cross_v3_v3v3(side_dir, axis, norm); normalize_v3(side_dir);
//
//		madd_v3_v3v3fl(segment0[0], center0, side_dir, radius0);
//		madd_v3_v3v3fl(segment0[1], center1, side_dir, radius1);
//
//		madd_v3_v3v3fl(segment1[0], center0, side_dir, -radius0);
//		madd_v3_v3v3fl(segment1[1], center1, side_dir, -radius1);
//	}
//}
///**
//* \return The number of point of interests
//* 0 - lines are colinear
//* 1 - lines are coplanar, i1 is set to intersection
//* 2 - i1 and i2 are the nearest points on line 1 (v1, v2) and line 2 (v3, v4) respectively
//*/
//int isect_segment_segment_plane_epsilon_v3(
//	const float v1[3], const float v2[3],
//	const float v3[3], const float v4[3], float i1[3], float i2[3],
//	const float epsilon)
//{
//	int ret;
//	ret = isect_line_line_epsilon_v3(v1, v2, v3, v4, i1, i2, epsilon);
//	if (ret == 1)
//	{
//		float seg0[3], dir_inter[3];
//		sub_v3_v3v3(seg0, v2, v1);
//		sub_v3_v3v3(dir_inter, i1, v1);
//		if (dot_v3v3(dir_inter, seg0) >= 0.0f && len_squared_v3(dir_inter) <= len_squared_v3(seg0))
//		{
//			sub_v3_v3v3(seg0, v4, v3);
//			sub_v3_v3v3(dir_inter, i1, v3);
//			if (dot_v3v3(dir_inter, seg0) >= 0.0f && len_squared_v3(dir_inter) <= len_squared_v3(seg0))
//			{
//				return 1;
//			}
//		}
//
//	}
//	return 0;
//}
///*calc intersection points between two cone-sphere's boundary segments on the plane defined by their axises*/
///*return the number of intersection points: 0, 1, 2*/
///*also return 0 when there are more than 2 intersection points*/
//static int calc_cone_sphere_intersection(
//	const float centers[3][3], float radius[3], float norm[3],
//	float out_inters[3][3])
//{
//	float  seg0[2][2][3], seg1[2][2][3];
//	float  tmp_inter0[3], tmp_inter1[3];
//	int i, j, status, inter_cnt;
//
//	calc_cone_sphere_segment(centers[0], radius[0], centers[1], radius[1], norm, seg0[0], seg0[1]);
//	calc_cone_sphere_segment(centers[2], radius[2], centers[1], radius[1], norm, seg1[0], seg1[1]);
//
//#ifdef _DEBUG
//	bool test = false;
//	if (test){
//		debug_segment_append(seg1[0][0], seg1[0][1]);
//		debug_segment_append(seg1[1][0], seg1[1][1]);
//		debug_segment_append(seg0[0][0], seg0[0][1]);
//		debug_segment_append(seg0[1][0], seg0[1][1]);
//	}
//#endif
//	/*calc intersection points between two cone-sphere on the plane: norm*/
//	inter_cnt = 0;
//	for (i = 0; i < 2; ++i){
//		for (j = 0; j < 2; ++j){
//			status = isect_segment_segment_plane_epsilon_v3(seg0[i][0], seg0[i][1], seg1[j][0], seg1[j][1], tmp_inter0, tmp_inter1, 0.001f);
//			if (status == 1)
//			{
//				/*we only consider at most 3 intersection points*/
//				if (inter_cnt == 3)
//					return 0;
//				copy_v3_v3(out_inters[inter_cnt++], tmp_inter0);
//			}
//		}
//	}
//	return inter_cnt;
//}
//
///*approximate the comic intersection section between two con-sphere as an ellipse*/
///*basis[0] is the larger axis*/
//static void calc_ellipse_section_interesction(
//	const float centers[3][3], float radius[3],
//	float origin[3], float basis[2][3], bool equalize_origin)
//{
//	float inters[3][3], dir0[3], dir1[3], norm[3], bisector[3], extreme[3], len;
//	int inter_cnt, i;
//
//	sub_v3_v3v3(dir0, centers[0], centers[1]);
//	sub_v3_v3v3(dir1, centers[2], centers[1]);
//	normalize_v3(dir0);
//	normalize_v3(dir1);
//	cross_v3_v3v3(norm, dir0, dir1); normalize_v3(norm);
//
//	inter_cnt = calc_cone_sphere_intersection(centers, radius, norm, inters);
//
//	if (1 == inter_cnt){
//		if (equalize_origin){
//			sub_v3_v3v3(bisector, inters[0], centers[1]); normalize_v3(bisector);
//
//			/*extreme point lies on sphere centers[1], in the opposite direction to bisector*/
//			madd_v3_v3v3fl(extreme, centers[1], bisector, -radius[1]);
//			mid_v3_v3v3(origin, extreme, inters[0]);
//			len = 0.5f* len_v3v3(extreme, inters[0]);
//
//			sub_v3_v3v3(basis[0], inters[0], origin);
//			copy_v3_v3(basis[1], norm); mul_v3_fl(basis[1], radius[1]);
//		}
//		else{
//			sub_v3_v3v3(basis[0], inters[0], centers[1]);
//			copy_v3_v3(origin, centers[1]);
//			copy_v3_v3(basis[1], norm); mul_v3_fl(basis[1], radius[1]);
//		}
//
//	}
//	else if (2 == inter_cnt){
//		mid_v3_v3v3(origin, inters[0], inters[1]);
//		sub_v3_v3v3(basis[0], inters[1], origin);
//		copy_v3_v3(basis[1], norm); mul_v3_fl(basis[1], radius[1]);
//		/*debug_point_coord_append(inters[0]);
//		debug_point_coord_append(inters[1]);*/
//	}
//	else if (3 == inter_cnt){
//		int longest_idx;
//		float longest_dst = FLT_MIN, dst;
//		add_v3_v3v3(bisector, dir0, dir1); normalize_v3(bisector);
//		/*choose the furthest point from centers[1], along the bisector*/
//		for (i = 0; i < 3; ++i){
//			sub_v3_v3v3(dir0, inters[i], centers[1]);
//			dst = dot_v3v3(dir0, bisector);
//			if (dst > longest_dst){
//				longest_dst = dst;
//				longest_idx = i;
//			}
//		}
//		if (equalize_origin){
//			/*now we come back to the same situation where there are only one intersection point*/
//			sub_v3_v3v3(bisector, inters[longest_idx], centers[1]); normalize_v3(bisector);
//			/*extreme point lies on sphere centers[1], in the opposite direction to bisector*/
//			madd_v3_v3v3fl(extreme, centers[1], bisector, -radius[1]);
//			mid_v3_v3v3(origin, extreme, inters[longest_idx]);
//			len = 0.5f* len_v3v3(extreme, inters[longest_idx]);
//
//			sub_v3_v3v3(basis[0], inters[longest_idx], origin);
//			copy_v3_v3(basis[1], norm); mul_v3_fl(basis[1], radius[1]);
//		}
//		else{
//			sub_v3_v3v3(basis[0], inters[longest_idx], centers[1]);
//			copy_v3_v3(origin, centers[1]);
//			copy_v3_v3(basis[1], norm); mul_v3_fl(basis[1], radius[1]);
//		}
//	}
//	else{
//		/*0 intersection points or strange cases*/
//		/*treat the comic section as a circle*/
//		copy_v3_v3(origin, centers[1]);
//		add_v3_v3v3(basis[0], dir0, dir1); normalize_v3(basis[0]); mul_v3_fl(basis[0], radius[1]);
//		copy_v3_v3(basis[1], norm); mul_v3_fl(basis[1], radius[1]);
//	}
//}
//
//static void generate_ellipse_verts(SkinData* data, const float origin[3], const float basis[2][3], BMVert** verts, int tot)
//{
//	BMesh* bm;
//	int i;
//	float step, coord[3];
//
//	bm = data->bm;
//	step = 2.0f* M_PI / tot;
//
//	for (i = 0; i < tot; ++i){
//		float angle = (float)i * step;
//		copy_v3_v3(coord, origin);
//		madd_v3_v3fl(coord, basis[0], cosf(angle));
//		madd_v3_v3fl(coord, basis[1], sinf(angle));
//		verts[i] = BM_vert_create(bm, coord, NULL, BM_CREATE_NOP);
//	}
//}
///*set skin vert reference for vertex set*/
//static void set_skin_vert_reference(BMVert** verts, int tot, int cd_vert_skin_off, int skin_v)
//{
//	int i;
//	for (i = 0; i < tot; ++i){
//		BM_ELEM_CD_SET_INT(verts[i], cd_vert_skin_off, skin_v);
//	}
//}
//static void evolve_connection_verts_to_cone_sphere(BMVert** verts, int tot, float skin_centers[3][3], float skin_rad[3])
//{
//
//#ifdef _DEBUG
//	if (len_v3v3(skin_centers[0], skin_centers[1]) < 0.1 * skin_rad[0] ||
//		len_v3v3(skin_centers[2], skin_centers[1]) < 0.1 * skin_rad[0])
//	{
//		return;
//	}
//#endif
//
//	const int ITER = 2;
//	int i, j;
//	float avg_proj[3], tmp[3];
//	BMVert* v;
//
//	for (j = 0; j < ITER; ++j)
//	{
//		for (i = 0; i < tot; ++i){
//			v = verts[i];
//			zero_v3(avg_proj);
//			project_on_cone_sphere_1(skin_centers[0], skin_rad[0], skin_centers[1], skin_rad[1], v->co, tmp);
//			add_v3_v3(avg_proj, tmp);
//			project_on_cone_sphere_1(skin_centers[2], skin_rad[2], skin_centers[1], skin_rad[1], v->co, tmp);
//			add_v3_v3(avg_proj, tmp);
//			mul_v3_fl(avg_proj, 0.5f);
//			copy_v3_v3(v->co, avg_proj);
//		}
//	}
//}
///*march along verts of edge*/
///*start from the vset0, and return the final vertex set in vset1*/
//static void march_on_edge(SkinData* data, SQMEdge* edge, const BMVert** vset0, int valence, BMVert** vset1, int* bridge_cache)
//{
//	int i, j;
//	int* verts_idx;
//	int tot, vstart_idx, vend_idx, cur_skin_idx, prev_skin_idx, next_skin_idx;
//	float centers[3][3], radius[3];
//	float origin[3], basis[2][3];
//	MVert* mverts;
//	MVertSkin* sverts;
//	SQMVert* sqm_verts = data->sqm_verts;
//	BMesh* bm;
//	BMVert** tmp_verts = BLI_array_alloca(tmp_verts, valence);
//	bm = data->bm;
//	mverts = data->verts;
//	sverts = data->sverts;
//	verts_idx = edge->verts;
//
//	vstart_idx = sqm_verts[edge->v1].vert;
//	vend_idx = sqm_verts[edge->v2].vert;
//
//	memcpy(tmp_verts, vset0, sizeof(BMVert*)*valence);
//	tot = edge->tot;
//	for (i = 0; i < tot; ++i){
//
//		prev_skin_idx = (i == 0) ? vstart_idx : edge->verts[i - 1];
//		cur_skin_idx = edge->verts[i];
//		next_skin_idx = (i == (tot - 1)) ? vend_idx : edge->verts[i + 1];
//
//		skin_vert_coord_radius_from_id(prev_skin_idx, mverts, sverts, centers[0], &radius[0]);
//		skin_vert_coord_radius_from_id(cur_skin_idx, mverts, sverts, centers[1], &radius[1]);
//		skin_vert_coord_radius_from_id(next_skin_idx, mverts, sverts, centers[2], &radius[2]);
//
//		calc_ellipse_section_interesction(centers, radius, origin, basis, true);
//
//		generate_ellipse_verts(data, origin, basis, vset1, valence);
//		set_verts_set_flag(vset1, valence, data->cd_vert_flag_offset, SKIN_BM_VERT_ON_CONNECTION);
//		set_skin_vert_reference(vset1, valence, data->cd_vert_skin_offset, cur_skin_idx);
//
//		evolve_connection_verts_to_cone_sphere(vset1, valence, centers, radius);
//
//		ensure_winding_order_hole_boundary_verts(tmp_verts, valence);
//		choose_bridge_order(data, bm, tmp_verts, vset1, valence, bridge_cache);
//		bridge_vert_set(data, bm, tmp_verts, vset1, bridge_cache, valence);
//
//		memcpy(tmp_verts, vset1, sizeof(BMVert*)*valence);
//	}
//}
//
//static void make_leaf_caption(SkinData* data, BMVert** verts, int tot, int skin_leaf_id, float leaf_center[3], float leaf_radius, float to_leaf_dir[3])
//{
//	/*extrude toward leaf center*/
//	int i, i_next, cd_face_flag_off, cd_vert_flag_off, cd_vert_skin_off;
//	float to_center[3];
//	const float scale = 0.2;
//	BMesh* bm;
//	BMFace* f;
//	BMVert* first_v, *last_v, *v;
//	bm = data->bm;
//
//	cd_face_flag_off = data->cd_face_flag_offset;
//	cd_vert_flag_off = data->cd_vert_flag_offset;
//	cd_vert_skin_off = data->cd_vert_skin_offset;
//	BMVert **extruded_verts = BLI_array_alloca(extruded_verts, tot);
//
//	/*create the first vertex*/
//	sub_v3_v3v3(to_center, leaf_center, verts[0]->co);
//	madd_v3_v3v3fl(to_center, verts[0]->co, to_center, 0.2);
//	first_v = BM_vert_create(bm, to_center, NULL, BM_CREATE_NOP);
//	last_v = first_v;
//	extruded_verts[0] = first_v;
//
//	/*extrude toward center*/
//	for (i = 1; i < tot; ++i){
//		sub_v3_v3v3(to_center, leaf_center, verts[i]->co);
//		madd_v3_v3v3fl(to_center, verts[i]->co, to_center, 0.2);
//		v = BM_vert_create(bm, to_center, NULL, BM_CREATE_NOP);
//		extruded_verts[i] = v;
//		f = BM_face_create_quad_tri(bm, verts[i - 1], verts[i], v, last_v, NULL, BM_CREATE_NOP);
//		set_face_flag(f, cd_face_flag_off, SKIN_BM_FACE_ON_LEAF);
//		last_v = v;
//	}
//
//	/*create the last quad*/
//	f = BM_face_create_quad_tri(bm, verts[tot - 1], verts[0], first_v, last_v, NULL, BM_CREATE_NOP);
//	set_face_flag(f, cd_face_flag_off, SKIN_BM_FACE_ON_LEAF);
//
//	/*create center verts*/
//	v = BM_vert_create(bm, leaf_center, NULL, BM_CREATE_NOP);
//	/*connect to the center*/
//	for (i = 0; i < tot; ++i){
//		f = BM_face_create_quad_tri(bm, extruded_verts[i], extruded_verts[(i + 1) % tot], v, NULL, NULL, BM_CREATE_NOP);
//		set_face_flag(f, cd_face_flag_off, SKIN_BM_FACE_ON_LEAF);
//	}
//
//	/*project on leaf sphere*/
//	set_vert_flag(v, cd_vert_flag_off, SKIN_BM_VERT_ON_LEAF);
//	BM_ELEM_CD_SET_INT(v, cd_vert_skin_off, skin_leaf_id);
//	project_on_sphere(leaf_center, leaf_radius, v->co, to_leaf_dir, v->co);
//	for (i = 0; i < tot; ++i){
//		set_vert_flag(extruded_verts[i], cd_vert_flag_off, SKIN_BM_VERT_ON_LEAF);
//		BM_ELEM_CD_SET_INT(extruded_verts[i], cd_vert_skin_off, skin_leaf_id);
//		project_on_sphere(leaf_center, leaf_radius, extruded_verts[i]->co, to_leaf_dir, extruded_verts[i]->co);
//	}
//}
//static void stich_branch_leaf_edge(SkinData* data, SQMEdge* edge)
//{
//	int valence;
//	float bary_center[3], leaf_center[3], leaf_radius, to_leaf_dir[3];
//	BMesh* bm;
//	BMVert* branch_path_v;
//	SQMVert* sqm_verts, *branch_sqm_v, *leaf_sqm_v;
//	MVertSkin* sverts;
//	MVert*	   mverts;
//	BLI_buffer_declare_static(BMVert*, vset0, BLI_BUFFER_NOP, 20);
//	BLI_buffer_declare_static(BMVert*, vset1, BLI_BUFFER_NOP, 20);
//	BLI_buffer_declare_static(int, bridge_order, BLI_BUFFER_NOP, 20);
//
//	/*one of path_vertex must be created earlier while building convex hull*/
//	if (!((edge->path_v1 && !edge->path_v2) || (!edge->path_v1 && edge->path_v2))) return;
//
//	/*init data*/
//	bm = data->bm;
//	sverts = data->sverts;
//	mverts = data->verts;
//	sqm_verts = data->sqm_verts;
//
//	/*ensure edge alway start from branch vertex v1*/
//	if (sqm_verts[edge->v2].flag & SQM_VERT_BRANCH){
//		SWAP(int, edge->v2, edge->v1);
//		if (edge->tot > 0){
//			std_algorithm_reverse(edge->verts, edge->verts + edge->tot);
//		}
//		edge->path_v1 = edge->path_v2;
//		edge->path_v2 = NULL;
//	}
//
//	/*now v1 is branch vertex and v2 is leaf vertex*/
//	BLI_assert(sqm_verts[edge->v1].flag & SQM_VERT_BRANCH);
//	BLI_assert(sqm_verts[edge->v2].flag & SQM_VERT_LEAF);
//
//	branch_sqm_v = &sqm_verts[edge->v1];
//	leaf_sqm_v = &sqm_verts[edge->v2];
//	branch_path_v = edge->path_v1;
//
//	copy_v3_v3(leaf_center, mverts[leaf_sqm_v->vert].co);
//	leaf_radius = skin_radius(sverts[leaf_sqm_v->vert].radius);
//
//	valence = BM_vert_edge_count(branch_path_v);
//	if (!(valence >= 4)) return;
//
//	BLI_buffer_resize(&vset0, valence);
//	BLI_buffer_resize(&vset1, valence);
//	BLI_buffer_resize(&bridge_order, valence);
//
//	/*remove adjacent faces around branch vertex*/
//	remove_adjacent_faces(data, branch_path_v, vset0.data, valence);
//	calc_bary_center(vset0.data, valence, bary_center);
//
//	/*marching on middle verts along edge*/
//	if (edge->tot > 0){
//		march_on_edge(data, edge, vset0.data, valence, vset1.data, bridge_order.data);
//		memcpy(vset0.data, vset1.data, sizeof(BMVert*)*valence);
//		calc_bary_center(vset0.data, valence, bary_center); /*recalc bary center*/
//	}
//
//	sub_v3_v3v3(to_leaf_dir, leaf_center, bary_center);
//	normalize_v3(to_leaf_dir);
//
//	generate_circle_verts(data, to_leaf_dir, leaf_center, leaf_radius, valence, vset1.data);
//	set_skin_vert_reference(vset1.data, valence, data->cd_vert_skin_offset, leaf_sqm_v->vert);
//	set_verts_set_flag(vset1.data, valence, data->cd_vert_flag_offset, SKIN_BM_VERT_ON_LEAF);
//
//	ensure_winding_order_hole_boundary_verts(vset0.data, valence);
//	choose_bridge_order(data, bm, vset0.data, vset1.data, valence, bridge_order.data);
//	bridge_vert_set(data, bm, vset0.data, vset1.data, bridge_order.data, valence);
//
//	ensure_winding_order_hole_boundary_verts(vset1.data, valence);
//	make_leaf_caption(data, vset1.data, valence, leaf_sqm_v->vert, leaf_center, leaf_radius, to_leaf_dir);
//}
//
//static void stich_branch_branch_edge(SkinData* data, SQMEdge* edge)
//{
//	int valence_v1, valence_v2, valence;
//	float bary_center0[3], bary_center1[3], dir[3];
//
//	BMesh* bm;
//	SQMVert* sqm_verts;
//	MVertSkin* sverts;
//	MVert*	   mverts;
//
//	BLI_buffer_declare_static(BMVert*, vset0, BLI_BUFFER_NOP, 20);
//	BLI_buffer_declare_static(BMVert*, vset1, BLI_BUFFER_NOP, 20);
//	BLI_buffer_declare_static(int, bridge_order, BLI_BUFFER_NOP, 20);
//
//	/*init data*/
//	bm = data->bm;
//	sverts = data->sverts;
//	mverts = data->verts;
//	sqm_verts = data->sqm_verts;
//
//	if (!edge->path_v1 || !edge->path_v2) return;
//
//	valence_v1 = BM_vert_edge_count(edge->path_v1);
//	valence_v2 = BM_vert_edge_count(edge->path_v2);
//
//	if (valence_v1 < 8 || valence_v2 < 8) return;
//	if (valence_v1 != valence_v2) return;
//	valence = valence_v1;
//
//	BLI_buffer_resize(&vset0, valence_v1);
//	BLI_buffer_resize(&vset1, valence_v1);
//	BLI_buffer_resize(&bridge_order, valence_v1);
//
//	/*remove adjacent faces around branch vertex*/
//	remove_adjacent_faces(data, edge->path_v1, vset0.data, valence);
//	calc_bary_center(vset0.data, valence, bary_center0);
//
//	/*marching on middle verts along edge*/
//	if (edge->tot > 0){
//		march_on_edge(data, edge, vset0.data, valence, vset1.data, bridge_order.data);
//		memcpy(vset0.data, vset1.data, sizeof(BMVert*)*valence);
//		calc_bary_center(vset0.data, valence, bary_center0); /*recalc bary center*/
//	}
//
//
//	remove_adjacent_faces(data, edge->path_v2, vset1.data, valence);
//	calc_bary_center(vset1.data, valence, bary_center1);
//
//	sub_v3_v3v3(dir, bary_center1, bary_center0);
//	normalize_v3(dir);
//
//	ensure_winding_order_hole_boundary_verts(vset0.data, valence);
//	choose_bridge_order(data, bm, vset0.data, vset1.data, valence, bridge_order.data);
//	bridge_vert_set(data, bm, vset0.data, vset1.data, bridge_order.data, valence);
//}
//
//static void stich_leaf_leaf_edge(SkinData* data, SQMEdge* edge)
//{
//#define LEAF_VALENCE 10
//
//	float bary_center[3], dir_to_leaf[3];
//	float leaf_center1[3], leaf_center2[3], leaf_radius1, leaf_radius2;
//	int sv1, sv2;
//	BMesh* bm;
//	SQMVert* sqm_verts;
//	MVertSkin* sverts;
//	MVert*	   mverts;
//
//	BLI_buffer_declare_static(BMVert*, vset0, BLI_BUFFER_NOP, 20);
//	BLI_buffer_declare_static(BMVert*, vset1, BLI_BUFFER_NOP, 20);
//	BLI_buffer_declare_static(int, bridge_order, BLI_BUFFER_NOP, 20);
//
//	/*init data*/
//	bm = data->bm;
//	sverts = data->sverts;
//	mverts = data->verts;
//	sqm_verts = data->sqm_verts;
//
//	if (edge->path_v1 || edge->path_v2) return;
//
//	sv1 = sqm_verts[edge->v1].vert;
//	sv2 = sqm_verts[edge->v2].vert;
//
//	skin_vert_coord_radius_from_id(sv1, mverts, sverts, leaf_center1, &leaf_radius1);
//	skin_vert_coord_radius_from_id(sv2, mverts, sverts, leaf_center2, &leaf_radius2);
//
//	/*next vertex of leaf_v1 along edge */
//	int next_v = (edge->tot > 0) ? edge->verts[0] : sqm_verts[edge->v2].vert;
//	sub_v3_v3v3(dir_to_leaf, leaf_center1, mverts[next_v].co); normalize_v3(dir_to_leaf);
//	generate_circle_verts(data, dir_to_leaf, leaf_center1, leaf_radius1, LEAF_VALENCE, vset0.data);
//	set_skin_vert_reference(vset0.data, LEAF_VALENCE, data->cd_vert_skin_offset, sv1);
//	set_verts_set_flag(vset0.data, LEAF_VALENCE, data->cd_vert_flag_offset, SKIN_BM_VERT_ON_LEAF);
//
//	/*create caption for leaf 1*/
//	make_leaf_caption(data, vset0.data, LEAF_VALENCE, sv1, leaf_center1, leaf_radius1, dir_to_leaf);
//	ensure_winding_order_hole_boundary_verts(vset0.data, LEAF_VALENCE);
//
//	copy_v3_v3(bary_center, leaf_center1);
//
//	/*marching along middle verts*/
//	if (edge->tot > 0){
//		march_on_edge(data, edge, vset0.data, LEAF_VALENCE, vset1.data, bridge_order.data);
//		memcpy(vset0.data, vset1.data, sizeof(BMVert*)*LEAF_VALENCE);
//		calc_bary_center(vset0.data, LEAF_VALENCE, bary_center); /*recalc bary center*/
//	}
//
//	/*stich with end leaf vertex*/
//	sub_v3_v3v3(dir_to_leaf, leaf_center2, bary_center); normalize_v3(dir_to_leaf);
//
//	generate_circle_verts(data, dir_to_leaf, leaf_center2, leaf_radius2, LEAF_VALENCE, vset1.data);
//	set_verts_set_flag(vset1.data, LEAF_VALENCE, data->cd_vert_flag_offset, SKIN_BM_VERT_ON_LEAF);
//	set_skin_vert_reference(vset1.data, LEAF_VALENCE, data->cd_vert_skin_offset, sv2);
//
//	ensure_winding_order_hole_boundary_verts(vset0.data, LEAF_VALENCE);
//	choose_bridge_order(data, bm, vset0.data, vset1.data, LEAF_VALENCE, bridge_order.data);
//	bridge_vert_set(data, bm, vset0.data, vset1.data, bridge_order.data, LEAF_VALENCE);
//
//	ensure_winding_order_hole_boundary_verts(vset1.data, LEAF_VALENCE);
//	make_leaf_caption(data, vset1.data, LEAF_VALENCE, sv2, leaf_center2, leaf_radius2, dir_to_leaf);
//}
//
//static void stich(SkinData* data)
//{
//	int i;
//	SQMEdge* sqm_edges = data->sqm_edges;
//	int tot_sqm_edges = data->tot_sqm_edges;
//	SQMVert* sqm_verts = data->sqm_verts;
//	for (i = 0; i < tot_sqm_edges; ++i){
//		SQMEdge* edge = &sqm_edges[i];
//		SQMVert* v1 = &sqm_verts[edge->v1];
//		SQMVert* v2 = &sqm_verts[edge->v2];
//
//		if ((v1->flag & SQM_VERT_BRANCH && v2->flag & SQM_VERT_LEAF) ||
//			(v2->flag & SQM_VERT_BRANCH && v1->flag & SQM_VERT_LEAF)){
//
//			stich_branch_leaf_edge(data, edge);
//		}
//		else if (v1->flag & SQM_VERT_BRANCH && v2->flag & SQM_VERT_BRANCH){
//			stich_branch_branch_edge(data, edge);
//		}
//		else if (v1->flag & SQM_VERT_LEAF && v2->flag & SQM_VERT_LEAF){
//			stich_leaf_leaf_edge(data, edge);
//		}
//	}
//}
//
///*return a new limb edge*/
//static BMEdge* split_limb_edge(SkinData* data, BMesh* bm, BMEdge* edge, BLI_Buffer* cache_faces, BLI_Buffer* cache_verts, int skin_v1, int skin_v2)
//{
//	BMLoop* le_iter = edge->l;
//	BMLoop* le_end = le_iter;
//	BMFace* f_end = le_end->f;
//	BMEdge* re;
//	BMLoop* rl;
//	BMVert* v;
//	int cnt = 0, i, cd_face_flag_off, cd_vert_flag_off;
//
//	cd_face_flag_off = data->cd_face_flag_offset;
//	cd_vert_flag_off = data->cd_vert_flag_offset;
//
//	do
//	{
//		if (cnt > cache_verts->count){
//			BLI_buffer_resize(cache_faces, cache_faces->count + 1);
//			BLI_buffer_resize(cache_verts, cache_verts->count + 1);
//		}
//
//		BMLoop *le_next = le_iter->radial_next->next->next;
//		BLI_buffer_at(cache_faces, BMFace*, cnt) = le_iter->radial_next->f;
//
//		v = BM_edge_split(bm, le_iter->e, le_iter->e->v1, &re, 0.5f);
//		BM_ELEM_CD_SET_INT(v, cd_vert_flag_off, 0); /*reset new vert's flag*/
//
//		BLI_buffer_at(cache_verts, BMVert*, cnt) = v;
//
//		set_vert_flag(v, cd_vert_flag_off, SKIN_BM_VERT_ON_LIMB);
//		encode_cone_sphere(data, v, skin_v1, skin_v2);
//
//		cnt++;
//		le_iter = le_next;
//
//	} while (le_iter->f != f_end);
//
//	for (i = 0; i < cnt; i++)
//	{
//		BMFace* f = BLI_buffer_at(cache_faces, BMFace*, i);
//		BMVert* v0 = BLI_buffer_at(cache_verts, BMVert*, i);
//		BMVert* v1 = BLI_buffer_at(cache_verts, BMVert*, (i + 1) % cnt);
//		BMLoop* l0 = BM_face_vert_share_loop(f, v0);
//		BMLoop* l1 = BM_face_vert_share_loop(f, v1);
//		if (l0 && l1){
//			BMFace* r_f = BM_face_split(bm, f, l0, l1, &rl, NULL, true);
//			BLI_assert(r_f);
//			set_face_flag(r_f, cd_face_flag_off, SKIN_BM_FACE_ON_LIMB);
//		}
//		else{
//			BLI_assert(false);
//		}
//	}
//
//	return re;
//}
//static void split_limb_edges(SkinData* data)
//{
//	BMEdge* edge, *new_e, *split_edge;
//	BMIter iter;
//	BMesh* bm;
//	int eflag, cd_edge_flag_offset, cd_vert_vert_offset;
//	bm = data->bm;
//	float skin_centers[2][3], skin_rad[2], tmp[3];
//	cd_edge_flag_offset = data->cd_edge_flag_offset;
//	cd_vert_vert_offset = data->cd_vert_skin_offset;
//
//	MVertSkin* sverts = data->sverts;
//	MVert* verts = data->verts;
//
//	BLI_buffer_declare(BMEdge*, split_edges, BLI_BUFFER_NOP);
//	BLI_buffer_declare(BMFace*, cache_faces, BLI_BUFFER_NOP);
//	BLI_buffer_declare(BMFace*, cache_verts, BLI_BUFFER_NOP);
//	BLI_buffer_resize(&cache_faces, 20);
//	BLI_buffer_resize(&cache_verts, 20);
//	BM_ITER_MESH(edge, &iter, bm, BM_EDGES_OF_MESH){
//		eflag = BM_ELEM_CD_GET_INT(edge, cd_edge_flag_offset);
//		if (eflag & SKIN_BM_EDGE_MARK_SPLIT){
//			BLI_buffer_append(&split_edges, BMEdge*, edge);
//		}
//	}
//
//	BLI_Stack* edge_stack = BLI_stack_new(sizeof(BMEdge*), "split_edge_stack");
//	int i;
//	for (i = 0; i < split_edges.count; ++i){
//
//		split_edge = BLI_buffer_at(&split_edges, BMEdge*, i);
//
//		/*calculate the edge length threshold for edge*/
//		int sv0 = BM_ELEM_CD_GET_INT(split_edge->v1, cd_vert_vert_offset);
//		int sv1 = BM_ELEM_CD_GET_INT(split_edge->v2, cd_vert_vert_offset);
//
//		skin_vert_coord_radius_from_v(split_edge->v1, verts, sverts, cd_vert_vert_offset, skin_centers[0], &skin_rad[0]);
//		skin_vert_coord_radius_from_v(split_edge->v2, verts, sverts, cd_vert_vert_offset, skin_centers[1], &skin_rad[1]);
//
//		float len_threshold = 0.5f*(skin_rad[0] + skin_rad[1]);
//
//		BLI_stack_push(edge_stack, &split_edge);
//		BMVert* old_verts[2], *new_vert;
//		while (!BLI_stack_is_empty(edge_stack)){
//
//			BLI_stack_pop(edge_stack, &split_edge);
//
//			old_verts[0] = split_edge->v1;
//			old_verts[1] = split_edge->v2;
//
//			split_limb_edge(data, bm, split_edge, &cache_faces, &cache_verts, sv0, sv1);
//
//			/*split_edge has been split. one of its verts is the new vertex*/
//			new_vert = (split_edge->v1 != old_verts[0] && split_edge->v1 != old_verts[1]) ? split_edge->v1 : split_edge->v2;
//
//			/*split further*/
//			split_edge = BM_edge_exists(old_verts[0], new_vert);
//			BLI_assert(split_edge);
//			if (BM_edge_calc_length(split_edge) > len_threshold)
//				BLI_stack_push(edge_stack, &split_edge);
//
//			split_edge = BM_edge_exists(old_verts[1], new_vert);
//			BLI_assert(split_edge);
//			if (BM_edge_calc_length(split_edge) > len_threshold)
//				BLI_stack_push(edge_stack, &split_edge);
//		}
//	}
//
//	BLI_stack_free(edge_stack);
//}
//static void project_limb_verts_on_cone_sphere(SkinData* data)
//{
//	BMEdge* e;
//	BMVert* v;
//	BMIter iter;
//	BMesh *bm;
//	float skin_centers[2][3], skin_rad[2], tmp[3];
//	int cd_vert_flag_offset, cd_vert_vert_offset, vflag, skin_v1, skin_v2;
//	MVert* mverts;
//	MVertSkin* sverts;
//
//	bm = data->bm;
//	cd_vert_flag_offset = data->cd_vert_flag_offset;
//	cd_vert_vert_offset = data->cd_vert_skin_offset;
//	mverts = data->verts;
//	sverts = data->sverts;
//
//	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH){
//		vflag = BM_ELEM_CD_GET_INT(v, cd_vert_flag_offset);
//		if (vflag & SKIN_BM_VERT_ON_LIMB){
//
//			decode_cone_sphere(data, v, &skin_v1, &skin_v2);
//			skin_vert_coord_radius_from_id(skin_v1, mverts, sverts, skin_centers[0], &skin_rad[0]);
//			skin_vert_coord_radius_from_id(skin_v2, mverts, sverts, skin_centers[1], &skin_rad[1]);
//
//			if (project_on_cone_sphere_1(skin_centers[0], skin_rad[0], skin_centers[1], skin_rad[1], v->co, tmp))
//				copy_v3_v3(v->co, tmp);
//		}
//	}
//}
//static int count_vert_node_degree(BMVert* v, int cd_vert_skin_offset)
//{
//	BMEdge* e;
//	BMVert* vv;
//	BMIter iter;
//	int skin_vert_id, cnt = 0;
//	skin_vert_id = BM_ELEM_CD_GET_INT(v, cd_vert_skin_offset);
//	BM_ITER_ELEM(e, &iter, v, BM_EDGES_OF_VERT){
//		vv = BM_edge_other_vert(e, v);
//		if (BM_ELEM_CD_GET_INT(vv, cd_vert_skin_offset) != skin_vert_id)
//			cnt++;
//	}
//	return cnt;
//}
//
//static void classify_branch_verts(SkinData* data)
//{
//	BMesh* bm;
//	BMVert* v;
//	BMIter iter;
//	int v_degree;
//	int cd_vert_flag_offset = data->cd_vert_flag_offset;
//	int cd_vert_vert_offset = data->cd_vert_skin_offset;
//
//	bm = data->bm;
//
//	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH){
//		int vflag = BM_ELEM_CD_GET_INT(v, cd_vert_flag_offset);
//		if (vflag & SKIN_BM_VERT_ON_BRANCH)
//		{
//			v_degree = count_vert_node_degree(v, cd_vert_vert_offset);
//			switch (v_degree)
//			{
//			case 0:
//				vflag |= SKIN_BM_VERT_DEGREE_0;
//				break;
//			case 1:
//				vflag |= SKIN_BM_VERT_DEGREE_1;
//				break;
//			case 2:
//				vflag |= SKIN_BM_VERT_DEGREE_2;
//				break;
//			case 3:
//				vflag |= SKIN_BM_VERT_DEGREE_3;
//				break;
//			default:
//				BLI_assert(false);
//				break;
//			}
//			BM_ELEM_CD_SET_INT(v, cd_vert_flag_offset, vflag);
//		}
//
//	}
//}
//
//
//
//static int solve_quadric_equation(float a, float b, float c, float* roots, float epsilon)
//{
//	if (fabsf(a) < epsilon){
//		if (fabsf(b) < epsilon){
//			return 0;
//		}
//		roots[0] = -c / b;
//		return 1;
//	}
//
//	float delta = b*b - 4 * a*c;
//	if (fabsf(delta) < epsilon){
//		roots[0] = -b / a / 2.0f;
//		return 1;
//	}
//	else if (delta < 0.0f){
//		return 0;
//	}
//
//	delta = sqrtf(delta);
//	a *= 2.0f;
//	roots[0] = (-b + delta) / a;
//	roots[1] = (-b - delta) / a;
//	return 2;
//}
//
///*ver0 should be on the longer ellipse axis*/
//static int isect_line_ellipse(
//	const float center[3], const float ver0[3], const float ver1[3],
//	const float p0[3], const float p1[3],
//	float inter0[3], float inter1[1])
//{
//	float dir[3], axis0[3], axis1[3], f1[3], f2[3], v1[3], v2[3];
//	float a2, b2;
//
//	sub_v3_v3v3(dir, p0, p1); normalize_v3(dir);
//	sub_v3_v3v3(axis0, center, ver0);
//	sub_v3_v3v3(axis1, center, ver1);
//	a2 = len_squared_v3(axis0);
//	b2 = len_squared_v3(axis1);
//
//	/*ensure that axis0 is the longer axis*/
//	if (a2 < b2){
//		swap_v3_v3(axis0, axis1);
//		swap_v3_v3(ver0, ver1);
//		SWAP(float, a2, b2);
//	}
//
//	normalize_v3(axis0);
//
//	madd_v3_v3v3fl(f1, center, axis0, sqrtf(a2 - b2));
//	zero_v3(f2); add_v3_v3v3(f2, center, center); sub_v3_v3(f2, f1);
//	sub_v3_v3v3(v1, p1, f1);
//	sub_v3_v3v3(v2, p1, f2);
//
//	float vv1 = dot_v3v3(v1, v1);
//	float vv2 = dot_v3v3(v2, v2);
//	float dv1 = dot_v3v3(dir, v1);
//	float dv2 = dot_v3v3(dir, v2);
//	float dd = dot_v3v3(dir, dir);
//	float v1v2 = dv1 - dv2;
//	float A = 4 * v1v2*v1v2 - 16 * a2*dd;
//	float B = 4 * v1v2*(vv1 - vv2) - 16 * a2*(dv1 + dv2);
//	float C = 16 * a2*a2 - 8 * a2*(vv1 + vv2) + (vv1 - vv2)*(vv1 - vv2);
//	float t[2];
//
//	int cnt = solve_quadric_equation(A, B, C, t, FLT_EPSILON);
//
//	if (cnt == 1){
//		madd_v3_v3v3fl(inter0, p1, dir, t[0]);
//		return 1;
//	}
//	else if (cnt == 2){
//		madd_v3_v3v3fl(inter0, p1, dir, t[0]);
//		madd_v3_v3v3fl(inter1, p1, dir, t[1]);
//		return 2;
//	}
//	else{
//		return 0;
//	}
//}
//#ifdef _DEBUG
//static void debug_test_ellipse_section(float org[3], float basic0[3], float basic1[1])
//{
//	int i;
//	float step, coord[3];
//	int tot = 50;
//	step = 2.0f* M_PI / tot;
//
//	for (i = 0; i < tot; ++i){
//		float angle = (float)i * step;
//		copy_v3_v3(coord, org);
//		madd_v3_v3fl(coord, basic0, cosf(angle));
//		madd_v3_v3fl(coord, basic1, sinf(angle));
//		//debug_point_coord_append(coord);
//	}
//}
//#endif
//
//static void project_on_comic_section(
//	const float centers[3][3], const float rad[3],
//	const float p[3], const float dir[3], float proj[3], bool equalize_comic_center)
//{
//	float comic_org[3], comic_basis[2][3], norm[3], ellipse_extreme0[3], ellipse_extreme1[3], line0[3], line1[3];
//	float p_on_comic[3], dir_on_comic[3], inter0[3], inter1[3], max_len;
//	int cnt_inter;
//
//	calc_ellipse_section_interesction(centers, rad, comic_org, comic_basis, equalize_comic_center);
//	cross_v3_v3v3(norm, comic_basis[0], comic_basis[1]);
//
//	project_v3_plane(p, norm, comic_org);
//	madd_v3_v3v3fl(dir_on_comic, dir, norm, -dot_v3v3(norm, dir)); /*dir - dir.dot(norm) * norm*/
//
//#ifdef _DEBUG
//	//float tmp[3];
//	//debug_point_coord_append(p);
//	//add_v3_v3v3(tmp, p, dir_on_comic);
//	//debug_segment_append(p, tmp);
//
//	//add_v3_v3v3(tmp, p, comic_basis[0]);
//	//debug_segment_append(p, tmp);
//	//add_v3_v3v3(tmp, p, comic_basis[1]);
//	//debug_segment_append(p, tmp);
//	//debug_test_ellipse_section(comic_org, comic_basis[0], comic_basis[1]);
//
//	//float test_len0 = len_v3(comic_basis[0]);
//	//float test_len1 = len_v3(comic_basis[1]);
//#endif
//
//	if (fabsf(len_squared_v3(comic_basis[0]) - len_squared_v3(comic_basis[1]))> FLT_EPSILON){
//		/*a real ellipse here*/
//		add_v3_v3v3(ellipse_extreme0, comic_org, comic_basis[0]);
//		add_v3_v3v3(ellipse_extreme1, comic_org, comic_basis[1]);
//
//		max_len = max(len_v3(comic_basis[0]), 1.0f);
//		madd_v3_v3v3fl(line0, p, dir_on_comic, 5.0f* max_len);
//		madd_v3_v3v3fl(line1, p, dir_on_comic, -5.0f* max_len);
//
//		cnt_inter = isect_line_ellipse(comic_org, ellipse_extreme0, ellipse_extreme1, line0, line1, inter0, inter1);
//
//		if (cnt_inter == 1){
//			copy_v3_v3(proj, inter0);
//		}
//		else if (cnt_inter == 2){
//			sub_v3_v3v3(line0, inter0, comic_org);
//			if (dot_v3v3(dir_on_comic, line0) > 0.0f){
//				copy_v3_v3(proj, inter0);
//			}
//			else{
//				copy_v3_v3(proj, inter1);
//			}
//		}
//		else{
//			/*unexpeceted case here*/
//			copy_v3_v3(proj, p);
//		}
//	}
//	else{
//		/*a circle*/
//		copy_v3_v3(proj, p);
//	}
//}
//
//
//static bool is_3_degree_vert_right_position(BMVert* v, BMVert* adj[3])
//{
//	const float threshold = M_PI / 12.0f;
//	float avg_angle, norm[3], edge[3][3];
//	int i, next, prev;
//	for (i = 0; i < 3; ++i){
//		sub_v3_v3v3(edge[i], adj[i]->co, v->co);
//		normalize_v3(edge[i]);
//	}
//
//	avg_angle = 0.0f;
//	for (i = 0; i < 3; ++i){
//		next = (i + 1) % 3;
//		prev = (i + 2) % 3;
//		cross_v3_v3v3(norm, edge[prev], edge[next]);
//		normalize_v3(norm);
//		avg_angle += M_PI_2 - angle_normalized_v3v3(norm, edge[i]);
//	}
//	avg_angle /= 3.0f;
//	return avg_angle > threshold;
//}
//static void evolve_vert_3_degree(SkinData* data)
//{
//	BMEdge* e;
//	BMVert* v, *vv;
//	BMIter iter, eiter;
//	BMesh* bm;
//	int cd_vert_flag_off;
//	int cd_vert_skin_off;
//	int center_skin, skin, cnt, i, j;
//	BMVert* adj_verts[3];
//	float skin_centers[3][3], skin_rad[3], tmp[3], avg_proj[3], proj_dir[3];
//	MVertSkin* sverts;
//	MVert* mverts;
//
//	cd_vert_flag_off = data->cd_vert_flag_offset;
//	cd_vert_skin_off = data->cd_vert_skin_offset;
//	sverts = data->sverts;
//	mverts = data->verts;
//	bm = data->bm;
//
//	BM_mesh_normals_update(bm);
//
//	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH){
//		int vflag = BM_ELEM_CD_GET_INT(v, cd_vert_flag_off);
//		if (vflag & SKIN_BM_VERT_ON_BRANCH && vflag & SKIN_BM_VERT_DEGREE_3){
//
//			center_skin = vert_skin_idx(v, cd_vert_skin_off);
//
//			/*collect adjacent skin sphere*/
//			cnt = 0;
//			zero_v3(proj_dir);
//			BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
//				vv = BM_edge_other_vert(e, v);
//				skin = vert_skin_idx(vv, cd_vert_skin_off);
//				if (center_skin != skin && cnt < 3){
//					adj_verts[cnt++] = vv;
//					sub_v3_v3v3(tmp, vv->co, v->co);
//					normalize_v3(tmp);
//					add_v3_v3(proj_dir, tmp);
//				}
//			}
//
//			normalize_v3(proj_dir);
//			if (dot_v3v3(proj_dir, v->no) < 0.0)
//				negate_v3(proj_dir);
//
//			zero_v3(avg_proj);
//			/*for each cone-sphere pair */
//			skin_vert_coord_radius_from_v(v, mverts, sverts, cd_vert_skin_off, skin_centers[1], &skin_rad[1]);
//			for (i = 0; i < 2; ++i)
//			{
//				skin_vert_coord_radius_from_v(adj_verts[i], mverts, sverts, cd_vert_skin_off, skin_centers[0], &skin_rad[0]);
//				for (j = i + 1; j < 3; ++j)
//				{
//					skin_vert_coord_radius_from_v(adj_verts[j], mverts, sverts, cd_vert_skin_off, skin_centers[2], &skin_rad[2]);
//					project_on_comic_section(skin_centers, skin_rad, v->co, proj_dir, tmp, true);
//					add_v3_v3(avg_proj, tmp);
//				}
//			}
//			mul_v3_fl(avg_proj, 1.0f / 3.0f);
//			copy_v3_v3(v->co, avg_proj);
//
//			/*fitting 1*/
//			const int ITER = 15;
//			float v_center[3], v_radius;
//
//			skin_vert_coord_radius_from_v(v, mverts, sverts, cd_vert_skin_off, v_center, &v_radius);
//			for (i = 0; i < 3; ++i){
//				skin_vert_coord_radius_from_v(adj_verts[i], mverts, sverts, cd_vert_skin_off, skin_centers[i], &skin_rad[i]);
//			}
//			for (j = 0; j < ITER; ++j){
//				zero_v3(avg_proj);
//				for (i = 0; i < 3; ++i){
//					project_on_cone_sphere_1(v_center, v_radius, skin_centers[i], skin_rad[i], v->co, tmp);
//					add_v3_v3(avg_proj, tmp);
//				}
//
//				mul_v3_fl(avg_proj, 1.0f / 3.0f);
//				copy_v3_v3(v->co, avg_proj);
//			}
//		}
//	}
//}
//static void smooth_vert_2_degree(SkinData* data)
//{
//	BMEdge* e;
//	BMVert* v, *vv;
//	BMIter iter, eiter;
//	BMesh* bm;
//	int cd_vert_flag_off;
//	int cd_vert_skin_off;
//	int center_skin, adj_skin, cnt, i, j;
//	float avg_coord[3];
//	MVertSkin* sverts;
//	MVert* mverts;
//
//	cd_vert_flag_off = data->cd_vert_flag_offset;
//	cd_vert_skin_off = data->cd_vert_skin_offset;
//	sverts = data->sverts;
//	mverts = data->verts;
//	bm = data->bm;
//	bool smooth_out;
//
//	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH){
//		int vflag = BM_ELEM_CD_GET_INT(v, cd_vert_flag_off);
//		if (vflag & SKIN_BM_VERT_ON_BRANCH && vflag & SKIN_BM_VERT_DEGREE_2){
//
//			center_skin = vert_skin_idx(v, cd_vert_skin_off);
//
//			cnt = 0;
//			smooth_out = false;
//			zero_v3(avg_coord);
//			BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
//				vv = BM_edge_other_vert(e, v);
//
//				adj_skin = vert_skin_idx(vv, cd_vert_skin_off);
//
//				/*adjacent vertex on the same skin node*/
//				if (center_skin == adj_skin){
//
//					/*only smooth out vertices adjacent to 3_degree verts on the same skin vert*/
//					if (BM_ELEM_CD_GET_INT(vv, cd_vert_flag_off) & SKIN_BM_VERT_DEGREE_3)
//						smooth_out = true;
//					add_v3_v3(avg_coord, vv->co);
//					cnt++;
//				}
//			}
//			if (smooth_out){
//				BLI_assert(cnt == 2);
//				mul_v3_fl(avg_coord, 0.5f);
//				copy_v3_v3(v->co, avg_coord);
//			}
//		}
//	}
//}
//static void evolve_vert_2_degree(SkinData* data)
//{
//	BMEdge* e;
//	BMVert* v, *vv;
//	BMIter iter, eiter;
//	BMesh* bm;
//	int cd_vert_flag_off;
//	int cd_vert_skin_off;
//	int center_skin, skin, cnt, i, j;
//	const int ITERATION = 10;
//	BMVert* adj_verts[2];
//	float skin_centers[3][3], skin_rad[3], edge_dirs[2][3], tmp[3], proj_dir[3];
//	MVertSkin* sverts;
//	MVert* mverts;
//
//	cd_vert_flag_off = data->cd_vert_flag_offset;
//	cd_vert_skin_off = data->cd_vert_skin_offset;
//	sverts = data->sverts;
//	mverts = data->verts;
//	bm = data->bm;
//
//	BM_mesh_normals_update(bm);
//
//	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH){
//		int vflag = BM_ELEM_CD_GET_INT(v, cd_vert_flag_off);
//		if (vflag & SKIN_BM_VERT_ON_BRANCH && vflag & SKIN_BM_VERT_DEGREE_2){
//
//			center_skin = vert_skin_idx(v, cd_vert_skin_off);
//
//			/*collect adjacent skin sphere*/
//			cnt = 0;
//			BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
//				vv = BM_edge_other_vert(e, v);
//				skin = vert_skin_idx(vv, cd_vert_skin_off);
//				if (center_skin != skin && cnt < 2){
//					adj_verts[cnt] = vv;
//					sub_v3_v3v3(edge_dirs[cnt], vv->co, v->co);
//					normalize_v3(edge_dirs[cnt]);
//					cnt++;
//				}
//			}
//
//			/*get skin infor*/
//			skin_vert_coord_radius_from_v(v, mverts, sverts, cd_vert_skin_off, skin_centers[1], &skin_rad[1]);
//			skin_vert_coord_radius_from_v(adj_verts[0], mverts, sverts, cd_vert_skin_off, skin_centers[0], &skin_rad[0]);
//			skin_vert_coord_radius_from_v(adj_verts[1], mverts, sverts, cd_vert_skin_off, skin_centers[2], &skin_rad[2]);
//
//			/*move to the ellipse intersection between two cone-sphere*/
//			{
//				if (dot_v3v3(edge_dirs[0], edge_dirs[1]) > 0.0f){
//					add_v3_v3v3(proj_dir, edge_dirs[0], edge_dirs[1]);
//					if (dot_v3v3(proj_dir, v->no) < 0.0f){
//						negate_v3(proj_dir);
//					}
//				}
//				else{
//					copy_v3_v3(proj_dir, v->no);
//				}
//
//				project_on_comic_section(skin_centers, skin_rad, v->co, proj_dir, v->co, true);
//			}
//
//			/*fitting using cone-sphere projection*/
//			{
//				for (i = 0; i < ITERATION; ++i)
//				{
//					zero_v3(proj_dir);
//
//					project_on_cone_sphere_1(skin_centers[1], skin_rad[1], skin_centers[0], skin_rad[0], v->co, tmp);
//					add_v3_v3(proj_dir, tmp);
//
//					project_on_cone_sphere_1(skin_centers[1], skin_rad[1], skin_centers[2], skin_rad[2], v->co, tmp);
//					add_v3_v3(proj_dir, tmp);
//
//					mul_v3_fl(proj_dir, 0.5f);
//					copy_v3_v3(v->co, proj_dir);
//				}
//			}
//		}
//	}
//}
//
//static void smooth_vert_1_degree(SkinData* data)
//{
//	BMEdge* e;
//	BMVert* v, *vv;
//	BMIter iter, eiter;
//	BMesh* bm;
//	int cd_vert_flag_off;
//	float avg_coord[3], weight, tot_weight, dir[3];
//	MVertSkin* sverts;
//	MVert* mverts;
//
//	cd_vert_flag_off = data->cd_vert_flag_offset;
//	sverts = data->sverts;
//	mverts = data->verts;
//	bm = data->bm;
//
//	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH){
//		int vflag = BM_ELEM_CD_GET_INT(v, cd_vert_flag_off);
//		if (vflag & SKIN_BM_VERT_ON_BRANCH && vflag & SKIN_BM_VERT_DEGREE_1){
//
//			zero_v3(avg_coord);
//			tot_weight = 0.0f;
//			BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
//				vv = BM_edge_other_vert(e, v);
//				sub_v3_v3v3(dir, v->co, vv->co);
//				weight = 1.0f / (1.0e-5 + len_v3(dir));
//				madd_v3_v3fl(avg_coord, vv->co, weight);
//				tot_weight += weight;
//			}
//
//			mul_v3_fl(avg_coord, 1.0f / tot_weight);
//			copy_v3_v3(v->co, avg_coord);
//		}
//	}
//}
//
//static void evolve_vert_1_degree(SkinData* data)
//{
//	BMEdge* e;
//	BMVert* v, *vv;
//	BMIter iter, eiter;
//	BMesh* bm;
//	int cd_vert_flag_off;
//	int cd_vert_skin_off, center_skin, adj_skin;
//	float centers[2][3], radius[2], tmp[3];
//	MVertSkin* sverts;
//	MVert* mverts;
//
//	cd_vert_flag_off = data->cd_vert_flag_offset;
//	cd_vert_skin_off = data->cd_vert_skin_offset;
//	sverts = data->sverts;
//	mverts = data->verts;
//	bm = data->bm;
//
//	const double gamma = 0.7;
//
//	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH){
//		int vflag = BM_ELEM_CD_GET_INT(v, cd_vert_flag_off);
//		if (vflag & SKIN_BM_VERT_ON_BRANCH && vflag & SKIN_BM_VERT_DEGREE_1){
//
//			adj_skin = -1;
//			center_skin = BM_ELEM_CD_GET_INT(v, cd_vert_skin_off);
//			BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
//				vv = BM_edge_other_vert(e, v);
//				int other_skin = BM_ELEM_CD_GET_INT(vv, cd_vert_skin_off);
//				if (center_skin != other_skin){
//					adj_skin = other_skin;
//					break;
//				}
//			}
//
//			if (adj_skin != -1){
//				skin_vert_coord_radius_from_v(v, mverts, sverts, cd_vert_skin_off, centers[0], &radius[0]);
//				skin_vert_coord_radius_from_v(vv, mverts, sverts, cd_vert_skin_off, centers[1], &radius[1]);
//				project_on_cone_sphere_1(centers[0], radius[0], centers[1], radius[1], v->co, tmp);
//				mul_v3_fl(v->co, 1.0f - gamma);
//				madd_v3_v3v3fl(v->co, v->co, tmp, gamma);
//			}
//		}
//	}
//}
//static void smooth_vert_0_degree(SkinData* data)
//{
//	BMEdge* e;
//	BMVert* v, *vv;
//	BMIter iter, eiter;
//	BMesh* bm;
//	int cd_vert_flag_off;
//	float avg_coord[3], tot_weight, weight, tmp[3];
//	MVertSkin* sverts;
//	MVert* mverts;
//
//	cd_vert_flag_off = data->cd_vert_flag_offset;
//	sverts = data->sverts;
//	mverts = data->verts;
//	bm = data->bm;
//
//	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH){
//		int vflag = BM_ELEM_CD_GET_INT(v, cd_vert_flag_off);
//		if (vflag & SKIN_BM_VERT_ON_BRANCH && vflag & SKIN_BM_VERT_DEGREE_0){
//
//			zero_v3(avg_coord);
//			tot_weight = 0.0f;
//			BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
//				vv = BM_edge_other_vert(e, v);
//				weight = 1.0f / (1.0e-5 + len_v3v3(v->co, vv->co));
//				madd_v3_v3fl(avg_coord, vv->co, weight);
//				tot_weight += weight;;
//			}
//
//			mul_v3_fl(avg_coord, 1.0f / tot_weight);
//			copy_v3_v3(v->co, avg_coord);
//		}
//	}
//}
//static void evolve(SkinData* data)
//{
//	const int ITERATION = 2;
//	int i;
//	classify_branch_verts(data);
//
//	evolve_vert_3_degree(data);
//
//	smooth_vert_2_degree(data);
//	evolve_vert_2_degree(data);
//
//	for (i = 0; i < ITERATION; ++i){
//		smooth_vert_1_degree(data);
//		evolve_vert_1_degree(data);
//		smooth_vert_0_degree(data);
//	}
//}
//
///*copied from old skin modifier*/
//static void quad_from_tris(BMEdge *e, BMFace *adj[2], BMVert *ndx[4])
//{
//	BMVert *tri[2][3];
//	BMVert *opp = NULL;
//	int i, j;
//
//	BLI_assert(adj[0]->len == 3 && adj[1]->len == 3);
//
//	BM_face_as_array_vert_tri(adj[0], tri[0]);
//	BM_face_as_array_vert_tri(adj[1], tri[1]);
//
//	/* Find what the second tri has that the first doesn't */
//	for (i = 0; i < 3; i++) {
//		if (tri[1][i] != tri[0][0] &&
//			tri[1][i] != tri[0][1] &&
//			tri[1][i] != tri[0][2])
//		{
//			opp = tri[1][i];
//			break;
//		}
//	}
//	BLI_assert(opp);
//
//	for (i = 0, j = 0; i < 3; i++, j++) {
//		ndx[j] = tri[0][i];
//		/* When the triangle edge cuts across our quad-to-be,
//		* throw in the second triangle's vertex */
//		if ((tri[0][i] == e->v1 || tri[0][i] == e->v2) &&
//			(tri[0][(i + 1) % 3] == e->v1 || tri[0][(i + 1) % 3] == e->v2))
//		{
//			j++;
//			ndx[j] = opp;
//		}
//	}
//}
//
/////*find edge help maximize the merge triangle count for edge around v*/
////static void merge_triange_mark_best_edge(BMVert* v, int cd_edge_flag_off)
////{
////	BMEdge* e;
////	BMIter eiter;
////	BMVert *quad[4];
////	BMFace *adj[2];
////	int eflag, merge_cnt_0, merge_cnt_1;
////	bool first_time;
////
////	/*mark edge around v whose adjacent triangle forming a convex quad*/
////	BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
////		/* Only care if the edge is used by exactly two triangles */
////		if (BM_edge_face_pair(e, &adj[0], &adj[1])){
////			if (adj[0]->len == 3 && adj[1]->len == 3) {
////
////				quad_from_tris(e, adj, quad);
////
////				if (is_quad_convex_v3(quad[0]->co, quad[1]->co, quad[2]->co, quad[3]->co)){
////					eflag = BM_ELEM_CD_GET_INT(e, cd_edge_flag_off);
////					eflag |= SKIN_BM_EDGE_CONVEX;
////					BM_ELEM_CD_SET_INT(e, cd_edge_flag_off, eflag);
////				}
////			}
////		}
////	}
////
////	/*emulate a deleting process to find a maixmum merging*/
////	merge_cnt_0 = 0;
////	BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
////		/* Only care if the edge is used by exactly two triangles and its quad is convex*/
////		if (BM_edge_face_pair(e, &adj[0], &adj[1]) &&
////			(adj[0]->len == 3 && adj[1]->len == 3) &&
////			!BM_elem_flag_test(adj[0], BM_ELEM_TAG) &&
////			!BM_elem_flag_test(adj[1], BM_ELEM_TAG) &&
////			(BM_ELEM_CD_GET_INT(e, cd_edge_flag_off) & SKIN_BM_EDGE_CONVEX)){
////
////			merge_cnt_0++;
////			BM_elem_flag_enable(adj[0], BM_ELEM_TAG);
////			BM_elem_flag_enable(adj[1], BM_ELEM_TAG);
////		}
////	}
////
////	/*clear tag*/
////	BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
////		if (BM_edge_face_pair(e, &adj[0], &adj[1])){
////			BM_elem_flag_disable(adj[0], BM_ELEM_TAG);
////			BM_elem_flag_disable(adj[1], BM_ELEM_TAG);
////		}
////	}
////
////
////	merge_cnt_1 = 0;
////	first_time = true;
////	BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
////		/*ignore the first edge to start from second edge*/
////		if (first_time){
////			first_time = false;
////			continue;
////		}
////		/* Only care if the edge is used by exactly two triangles and its quad is convex*/
////		if (BM_edge_face_pair(e, &adj[0], &adj[1]) &&
////			(adj[0]->len == 3 && adj[1]->len == 3) &&
////			!BM_elem_flag_test(adj[0], BM_ELEM_TAG) &&
////			!BM_elem_flag_test(adj[1], BM_ELEM_TAG) &&
////			(BM_ELEM_CD_GET_INT(e, cd_edge_flag_off) & SKIN_BM_EDGE_CONVEX)){
////
////			merge_cnt_1++;
////			BM_elem_flag_enable(adj[0], BM_ELEM_TAG);
////			BM_elem_flag_enable(adj[1], BM_ELEM_TAG);
////		}
////	}
////
////	first_time = (merge_cnt_1 > merge_cnt_0) ? true : false;
////
////	/*clear tag*/
////	BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
////		if (BM_edge_face_pair(e, &adj[0], &adj[1])){
////			BM_elem_flag_disable(adj[0], BM_ELEM_TAG);
////			BM_elem_flag_disable(adj[1], BM_ELEM_TAG);
////		}
////	}
////
////	BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
////		/*ignore the first edge to start from second edge*/
////		if (first_time){
////			first_time = false;
////			continue;
////		}
////
////		eflag = BM_ELEM_CD_GET_INT(e, cd_edge_flag_off);
////		/* Only care if the edge is used by exactly two triangles and its quad is convex*/
////		if (BM_edge_face_pair(e, &adj[0], &adj[1]) &&
////			(adj[0]->len == 3 && adj[1]->len == 3) &&
////			!BM_elem_flag_test(adj[0], BM_ELEM_TAG) &&
////			!BM_elem_flag_test(adj[1], BM_ELEM_TAG) &&
////			eflag & SKIN_BM_EDGE_CONVEX){
////			/*mark this edge for merging*/
////			eflag |= SKIN_BM_EDGE_TRIANGLE_MERGE;
////			BM_elem_flag_enable(adj[0], BM_ELEM_TAG);
////			BM_elem_flag_enable(adj[1], BM_ELEM_TAG);
////		}
////	}
////
////	BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
////		if (BM_edge_face_pair(e, &adj[0], &adj[1])){
////			BM_elem_flag_disable(adj[0], BM_ELEM_TAG);
////			BM_elem_flag_disable(adj[1], BM_ELEM_TAG);
////		}
////	}
////}
//
///*merge triangle around 0_degree verts on branch*/
///*this function must be called after revolve step since*/
//static void merge_branch_triangle(SkinData* data)
//{
//	BMEdge* e;
//	BMVert* v, *vv;
//	BMIter iter, eiter;
//	BMesh* bm;
//	int cd_vert_flag_off, cd_edge_flag_off, eflag;
//	BMVert *quad[4];
//	BMFace *adj[2];
//
//	cd_vert_flag_off = data->cd_vert_flag_offset;
//	cd_edge_flag_off = data->cd_edge_flag_offset;
//	bm = data->bm;
//
//	BM_mesh_normals_update(bm);
//
//	BM_mesh_elem_hflag_disable_all(bm, BM_FACE | BM_EDGE, BM_ELEM_TAG, false);
//
//	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH){
//		int vflag = BM_ELEM_CD_GET_INT(v, cd_vert_flag_off);
//		if (vflag & (SKIN_BM_VERT_ON_BRANCH | SKIN_BM_VERT_DEGREE_0)){
//
//			/*mark edge around v whose adjacent triangle forming a convex quad*/
//			BM_ITER_ELEM(e, &eiter, v, BM_EDGES_OF_VERT){
//				/* Only care if the edge is used by exactly two triangles */
//				if (BM_edge_face_pair(e, &adj[0], &adj[1]) &&
//					(adj[0]->len == 3 && adj[1]->len == 3) &&
//					!BM_elem_flag_test(adj[0], BM_ELEM_TAG) &&
//					!BM_elem_flag_test(adj[1], BM_ELEM_TAG)){
//
//					quad_from_tris(e, adj, quad);
//
//					if (is_quad_convex_v3(quad[0]->co, quad[1]->co, quad[2]->co, quad[3]->co)){
//						BM_face_create_quad_tri(bm, quad[0], quad[1], quad[2], quad[3], NULL, BM_CREATE_NOP);
//						BM_elem_flag_enable(adj[0], BM_ELEM_TAG);
//						BM_elem_flag_enable(adj[1], BM_ELEM_TAG);
//						BM_elem_flag_enable(e, BM_ELEM_TAG);
//					}
//				}
//			}
//		}
//	}
//
//	BM_mesh_delete_hflag_tagged(bm, BM_ELEM_TAG, BM_EDGE | BM_FACE);
//}
///*find the first original loop (existed before face' edges splitted)*/
//static BMLoop* find_first_loop(BMFace* face)
//{
//	BMLoop *l_iter, *l_first, *l_prev;
//	float dir_prev[3], dir_next[3], dot;
//	l_iter = l_first = BM_FACE_FIRST_LOOP(face);
//	do {
//		sub_v3_v3v3(dir_prev, l_iter->v->co, l_iter->prev->v->co);
//		sub_v3_v3v3(dir_next, l_iter->next->v->co, l_iter->v->co);
//		normalize_v3(dir_next);
//		normalize_v3(dir_prev);
//		dot = dot_v3v3(dir_prev, dir_next); /*not collinear*/
//		if (dot < 1.0 - FLT_EPSILON)
//		{
//			return l_iter;
//		}
//	} while ((l_iter = l_iter->next) != l_first);
//
//	return NULL;
//}
//
//static BMVert* split_face_catmull(SkinData* data, BMFace* face)
//{
//	BMesh* bm;
//	float centroid[3];
//	BMVert* v_quad[4], *v_centroid;
//	BMEdge* e_quad[4];
//	BMFace* ff;
//	BMLoop *l_iter, *l_first;
//	bm = data->bm;
//
//	BLI_assert(face->len % 2 == 0);
//
//	BM_face_calc_center_mean(face, centroid);
//	v_centroid = BM_vert_create(bm, centroid, NULL, BM_CREATE_NOP);
//
//	l_iter = l_first = find_first_loop(face);
//	if (l_iter == NULL) return;
//
//	do {
//		v_quad[0] = l_iter->prev->v;
//		v_quad[1] = l_iter->v;
//		v_quad[2] = l_iter->next->v;
//		v_quad[3] = v_centroid;
//
//		bm_edges_from_quad(bm, v_quad, e_quad);
//
//		ff = BM_face_create(bm, v_quad, e_quad, 4, face, BM_CREATE_NOP);
//		BM_elem_flag_enable(ff, BM_ELEM_TAG);/*tag new face to avoid splitting again*/
//
//	} while ((l_iter = l_iter->next->next) != l_first);
//
//	BM_face_kill(bm, face);
//
//	return v_centroid;
//}
//
//static BMVert* get_vert_on_limb(BMFace* f, int cd_vert_flag_off)
//{
//	BMLoop* l_first, *l_iter;
//	l_first = l_iter = BM_FACE_FIRST_LOOP(f);
//	do
//	{
//		if (check_vert_flag(l_iter->v, cd_vert_flag_off, SKIN_BM_VERT_ON_LIMB))
//		{
//			return l_iter->v;
//		}
//	} while ((l_iter = l_iter->next) != l_first);
//
//	return NULL;
//}
//
///*define type for a new vertex on edge*/
///*set limb vertex to sample_limb_v in case of limb edge*/
//static int classify_vert_type_from_edge(BMEdge* e, int cd_vert_flag_off, BMVert** sample_limb_v)
//{
//	int flag_v1 = BM_ELEM_CD_GET_INT(e->v1, cd_vert_flag_off);
//	int flag_v2 = BM_ELEM_CD_GET_INT(e->v2, cd_vert_flag_off);
//	/*both edge's verts on branch ==> new vertex must be branch vertex*/
//	if (flag_v1 & SKIN_BM_VERT_ON_BRANCH && flag_v2 & SKIN_BM_VERT_ON_BRANCH){
//		return SKIN_BM_VERT_ON_BRANCH;
//	}
//	else if (flag_v1 & SKIN_BM_VERT_ON_CONNECTION && flag_v2 & SKIN_BM_VERT_ON_CONNECTION){
//		return SKIN_BM_VERT_ON_CONNECTION;
//	}
//	else if (flag_v1 & SKIN_BM_VERT_ON_LEAF && flag_v2 & SKIN_BM_VERT_ON_LEAF){
//		return SKIN_BM_VERT_ON_LEAF;
//	}
//	else{
//		/*must a limb vertex here*/
//		if (flag_v1 & SKIN_BM_VERT_ON_LIMB){
//			*sample_limb_v = e->v1;
//			return SKIN_BM_VERT_ON_LIMB;
//		}
//		else if (flag_v2 & SKIN_BM_VERT_ON_LIMB){
//			*sample_limb_v = e->v2;
//			return SKIN_BM_VERT_ON_LIMB;
//		}
//		else{
//			return -1;
//		}
//	}
//}
//
//static int classify_vert_type_from_face(BMFace* f, int cd_face_flag_off, int cd_vert_flag_off, BMVert** sample_limb_v)
//{
//	int fflag = BM_ELEM_CD_GET_INT(f, cd_face_flag_off);
//	if (fflag & SKIN_BM_FACE_ON_LIMB){
//		*sample_limb_v = get_vert_on_limb(f, cd_vert_flag_off);
//		return SKIN_BM_VERT_ON_LIMB;
//	}
//	else if (fflag & SKIN_BM_FACE_ON_LEAF){
//		return SKIN_BM_VERT_ON_LEAF;
//	}
//	else{
//		return  SKIN_BM_VERT_ON_BRANCH;
//	}
//}
//static void subdivide_mesh(SkinData* data)
//{
//	BMesh* bm;
//	BMVert* v, *v_on_limb;
//	BMFace* f;
//	BMEdge* e, *r_e;
//	BMIter iter;
//	bm = data->bm;
//	bool face_on_limb;
//	int cd_vert_flag_off, cd_vert_skin_off, cd_face_flag_off, cd_vert_sqm_off;
//	int cone_sphere, v_type;
//	bool v_on_branch;
//
//	cd_vert_flag_off = data->cd_vert_flag_offset;
//	cd_vert_skin_off = data->cd_vert_skin_offset;
//
//	cd_face_flag_off = data->cd_face_flag_offset;
//
//	/*split edge*/
//	BM_mesh_elem_hflag_disable_all(bm, BM_EDGE | BM_FACE, BM_ELEM_TAG, false);
//
//	BM_ITER_MESH(e, &iter, bm, BM_EDGES_OF_MESH){
//		/*ignore splitted edges*/
//		if (BM_elem_flag_test(e, BM_ELEM_TAG)) continue;
//
//		/*find type for new vertex from edge*/
//		v_on_limb = NULL;
//		v_type = classify_vert_type_from_edge(e, cd_vert_flag_off, &v_on_limb);
//		BLI_assert(v_type != -1);
//
//		/*create a new vertex*/
//		v = BM_edge_split(bm, e, e->v1, &r_e, 0.5f);
//		BM_ELEM_CD_SET_INT(v, cd_vert_flag_off, 0); /*reset new vert's flag*/
//
//		if (v_on_limb){
//			BM_ELEM_CD_SET_INT(v, cd_vert_skin_off, BM_ELEM_CD_GET_INT(v_on_limb, cd_vert_skin_off));
//		}
//		else{
//			BM_ELEM_CD_SET_INT(v, cd_vert_skin_off, BM_ELEM_CD_GET_INT(e->v1, cd_vert_skin_off));
//		}
//		if (v_type != -1){
//			set_vert_flag(v, cd_vert_flag_off, v_type);
//		}
//
//		/*mark new edges to avoid splitting again*/
//		BM_elem_flag_enable(e, BM_ELEM_TAG);
//		BM_elem_flag_enable(r_e, BM_ELEM_TAG);
//	}
//
//	/*split face*/
//	BM_ITER_MESH(f, &iter, bm, BM_FACES_OF_MESH){
//		/*ignore splitted edges*/
//		if (BM_elem_flag_test(f, BM_ELEM_TAG)) continue;
//
//		v_on_limb = NULL;
//		v_type = classify_vert_type_from_face(f, cd_face_flag_off, cd_vert_flag_off, &v_on_limb);
//
//		v = split_face_catmull(data, f);
//		BM_ELEM_CD_SET_INT(v, cd_vert_flag_off, 0); /*reset new vert's flag*/
//
//		if (v_type == SKIN_BM_VERT_ON_LIMB && v_on_limb != NULL){
//			set_vert_flag(v, cd_vert_flag_off, SKIN_BM_VERT_ON_LIMB);
//			BM_ELEM_CD_SET_INT(v, cd_vert_skin_off, BM_ELEM_CD_GET_INT(v_on_limb, cd_vert_skin_off));
//
//		}
//		else{
//			/*v is on branch*/
//			BM_ELEM_CD_SET_INT(v, cd_vert_skin_off, BM_ELEM_CD_GET_INT((BM_FACE_FIRST_LOOP(f)->v), cd_vert_skin_off));
//			set_vert_flag(v, cd_vert_flag_off, v_type);
//		}
//	}
//
//}
//
//static void evolve_verts_after_subdivision(SkinData* data)
//{
//	BMEdge* e;
//	BMVert* v;
//	BMIter iter, eiter;
//	BMesh* bm;
//	int cd_vert_flag_off, cd_vert_skin_off;
//	int cnt, i, it, skin_center_idx, skin_adj_idx;
//	const int ITERATION = 20;
//	float cone_sphere_center[2][3], cone_sphere_rad[2], avg_proj[3], tmp[3], tmp1[3];
//	MVertSkin* sverts;
//	MVert* mverts;
//	MeshElemMap* emap;
//	MEdge *medges;;
//
//	cd_vert_flag_off = data->cd_vert_flag_offset;
//	cd_vert_skin_off = data->cd_vert_skin_offset;
//	sverts = data->sverts;
//	mverts = data->verts;
//	medges = data->edges;
//	emap = data->emap;
//	bm = data->bm;
//
//	BM_ITER_MESH(v, &iter, bm, BM_VERTS_OF_MESH){
//		int vflag = BM_ELEM_CD_GET_INT(v, cd_vert_flag_off);
//
//		if (vflag & SKIN_BM_VERT_ON_BRANCH || vflag  & SKIN_BM_VERT_ON_CONNECTION){
//
//			skin_center_idx = BM_ELEM_CD_GET_INT(v, cd_vert_skin_off);
//			skin_vert_coord_radius_from_id(skin_center_idx, mverts, sverts, cone_sphere_center[0], &cone_sphere_rad[0]);
//
//			/*evolve to the ellipse section by project on adjacent cone-sphere*/
//			for (it = 0; it < ITERATION; ++it){
//				zero_v3(avg_proj);
//				cnt = 0;
//				for (i = 0; i < emap[skin_center_idx].count; ++i){
//					MEdge* me = &medges[emap[skin_center_idx].indices[i]];
//
//					skin_adj_idx = BKE_mesh_edge_other_vert(me, skin_center_idx);
//					skin_vert_coord_radius_from_id(skin_adj_idx, mverts, sverts, cone_sphere_center[1], &cone_sphere_rad[1]);
//
//					if (project_on_cone_sphere_1(
//						cone_sphere_center[0], cone_sphere_rad[0],
//						cone_sphere_center[1], cone_sphere_rad[1], v->co, tmp)){
//						add_v3_v3(avg_proj, tmp);
//						cnt++;
//					}
//				}
//				if (cnt > 0){
//					mul_v3_fl(avg_proj, 1.0f / cnt);
//					copy_v3_v3(v->co, avg_proj);
//				}
//				else{
//				}
//			}
//		}
//		else if (vflag & SKIN_BM_VERT_ON_LEAF){
//			skin_center_idx = BM_ELEM_CD_GET_INT(v, cd_vert_skin_off);
//			skin_vert_coord_radius_from_id(skin_center_idx, mverts, sverts, cone_sphere_center[0], &cone_sphere_rad[0]);
//			sub_v3_v3v3(tmp, v->co, cone_sphere_center[0]);
//
//			/*only project point inside sphere*/
//			sub_v3_v3v3(tmp1, v->co, cone_sphere_center[0]);
//			if (dot_v3v3(tmp1, tmp1) < 0.99 * (cone_sphere_rad[0] * cone_sphere_rad[0]))
//				project_on_sphere(cone_sphere_center[0], cone_sphere_rad[0], v->co, tmp, v->co);
//		}
//	}
//}
//
//static void skin_set_orig_indices(DerivedMesh *dm)
//{
//	int *orig, totpoly;
//
//	totpoly = dm->getNumPolys(dm);
//	orig = CustomData_add_layer(&dm->polyData, CD_ORIGINDEX,
//		CD_CALLOC, NULL, totpoly);
//	fill_vn_i(orig, totpoly, ORIGINDEX_NONE);
//}
//
//static DerivedMesh *final_skin(KSkinModifierData *smd,
//	DerivedMesh *origdm)
//{
//	DerivedMesh *result;
//	int i = 0;
//	SkinData data;
//	data.sverts = CustomData_get_layer(&origdm->vertData, CD_MVERT_SKIN);
//	data.verts = origdm->getVertArray(origdm);
//	data.edges = origdm->getEdgeArray(origdm);
//	data.totverts = origdm->getNumVerts(origdm);
//	data.totedges = origdm->getNumEdges(origdm);
//
//	BLI_buffer_declare(float[3], cache_coord_set_quadbridge0, BLI_BUFFER_NOP);
//	BLI_buffer_declare(float[3], cache_coord_set_quadbridge1, BLI_BUFFER_NOP);
//	data._cache_coord_quad_bridge0 = &cache_coord_set_quadbridge0;
//	data._cache_coord_quad_bridge1 = &cache_coord_set_quadbridge1;
//
//	debug_init_data(&data);
//
//	BM_data_layer_add(data.bm, &data.bm->vdata, CD_PROP_INT);
//	BM_data_layer_add(data.bm, &data.bm->vdata, CD_PROP_INT);
//	BM_data_layer_add(data.bm, &data.bm->vdata, CD_PROP_INT);
//
//	data.cd_vert_skin_offset = CustomData_get_n_offset(&data.bm->vdata, CD_PROP_INT, 0);
//	data.cd_vert_flag_offset = CustomData_get_n_offset(&data.bm->vdata, CD_PROP_INT, 1);
//
//	BM_data_layer_add(data.bm, &data.bm->edata, CD_PROP_INT);
//	data.cd_edge_flag_offset = CustomData_get_offset(&data.bm->edata, CD_PROP_INT);
//
//	BM_data_layer_add(data.bm, &data.bm->pdata, CD_PROP_INT);
//	data.cd_face_flag_offset = CustomData_get_offset(&data.bm->pdata, CD_PROP_INT);
//
//	BKE_mesh_vert_edge_map_create(&data.emap, &data.emapmem, data.edges, data.totverts, data.totedges);
//
//	build_sqm_graph(&data);
//
//	build_polyhedra(smd, &data);
//
//	isolate_path_verts(&data);
//
//	split(&data);
//
//	equalize_valence(&data);
//
//	stich(&data);
//
//	evolve(&data);
//
//	merge_branch_triangle(&data);
//
//	split_limb_edges(&data);
//
//	subdivide_mesh(&data);
//
//	project_limb_verts_on_cone_sphere(&data);
//
//	evolve_verts_after_subdivision(&data);
//
//	//result = CDDM_from_bmesh(data.bm, false);
//
//	//CDDM_calc_edges(result);
//	//result->dirty |= DM_DIRTY_NORMALS;
//
//	//skin_set_orig_indices(result);
//
//	MEM_freeN(data.sqm_edge_vert_mem);
//	MEM_freeN(data.sqm_edges);
//	MEM_freeN(data.emap);
//	MEM_freeN(data.emapmem);
//	MEM_freeN(data.sqm_emap);
//	MEM_freeN(data.sqm_emapmem);
//	BLI_buffer_free(&cache_coord_set_quadbridge0);
//	BLI_buffer_free(&cache_coord_set_quadbridge1);
//
//	return NULL;
//
//}
//static void initData(ModifierData *md)
//{
//	SkinModifierData *smd = (SkinModifierData *)md;
//
//	/* Enable in editmode by default */
//	md->mode |= eModifierMode_Editmode;
//}
//static CustomDataMask requiredDataMask(Object *UNUSED(ob),
//	ModifierData *UNUSED(md))
//{
//	return CD_MASK_MVERT_SKIN;
//}
//
//static void copyData(ModifierData *md, ModifierData *target)
//{
//#if 0
//	SkinModifierData *smd = (SkinModifierData *)md;
//	SkinModifierData *tsmd = (SkinModifierData *)target;
//#endif
//	modifier_copyData_generic(md, target);
//}
//
//static DerivedMesh *applyModifier(ModifierData *md,
//	Object *UNUSED(ob),
//	DerivedMesh *dm,
//	ModifierApplyFlag UNUSED(flag))
//{
//	DerivedMesh *result;
//
//	/* Skin node layer is required */
//	if (!CustomData_get_layer(&dm->vertData, CD_MVERT_SKIN))
//		return dm;
//
//	KSkinModifierData* smd = md;
//	smd->debug_draw_function = debug_draw_kskin;
//	result = final_skin(md, dm);
//
//	return dm;
//}
//
//ModifierTypeInfo modifierType_KSkin = {
//	/* name */              "KSkin",
//	/* structName */        "KSkinModifierData",
//	/* structSize */        sizeof(KSkinModifierData),
//	/* type */              eModifierTypeType_Constructive,
//	/* flags */             eModifierTypeFlag_AcceptsMesh | eModifierTypeFlag_SupportsEditmode,
//
//	/* copyData */          copyData,
//	/* deformVerts */       NULL,
//	/* deformMatrices */    NULL,
//	/* deformVertsEM */     NULL,
//	/* deformMatricesEM */  NULL,
//	/* applyModifier */     applyModifier,
//	/* applyModifierEM */   NULL,
//	/* initData */          initData,
//	/* requiredDataMask */  requiredDataMask,
//	/* freeData */          NULL,
//	/* isDisabled */        NULL,
//	/* updateDepgraph */    NULL,
//	/* dependsOnTime */     NULL,
//	/* dependsOnNormals */  NULL,
//	/* foreachObjectLink */ NULL,
//	/* foreachIDLink */     NULL,
//	/* foreachTexLink */    NULL,
//};