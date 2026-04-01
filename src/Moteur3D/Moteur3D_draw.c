#include "Moteur3D.h"
#include <math.h>
#include <stdlib.h>

//MARK: Internal Types

typedef struct {
      float x, y, z, w;
} Vect4;

//MARK: Internal Projection Helpers

static inline Vect4 m3d_transform_vec4(const Matrice* matrix, const Vect4* point){
      const float* m = (const float*)matrix->data;

      return (Vect4){
            m[0] * point->x + m[1] * point->y + m[2] * point->z + m[3] * point->w,
            m[4] * point->x + m[5] * point->y + m[6] * point->z + m[7] * point->w,
            m[8] * point->x + m[9] * point->y + m[10] * point->z + m[11] * point->w,
            m[12] * point->x + m[13] * point->y + m[14] * point->z + m[15] * point->w
      };
}

static inline bool m3d_is_ndc_visible(const Vect3* ndc_point){
      return ndc_point->x >= -1.0f && ndc_point->x <= 1.0f
              && ndc_point->y >= -1.0f && ndc_point->y <= 1.0f;
}

static Vect4 m3d_world_to_view(const Moteur3D* moteur, const Vect3* world_point){
      Vect4 point = {world_point->x, world_point->y, world_point->z, 1.0f};
      return m3d_transform_vec4(&moteur->view, &point);
}

static Vect4 m3d_view_to_clip(const Moteur3D* moteur, const Vect4* view_point){
      return m3d_transform_vec4(&moteur->proj, view_point);
}

static bool m3d_clip_to_ndc(const Vect4* clip_point, Vect3* ndc_point){
      if(fabsf(clip_point->w) < 0.00001f){
            *ndc_point = (Vect3){0.0f, 0.0f, 0.0f};
            return false;
      }

      ndc_point->x = clip_point->x / clip_point->w;
      ndc_point->y = clip_point->y / clip_point->w;
      ndc_point->z = clip_point->z / clip_point->w;
      return true;
}

static void m3d_ndc_to_screen(const Moteur3D* moteur, const Vect3* ndc_point, int* screen_x, int* screen_y){
      *screen_x = (int)((ndc_point->x * 0.5f + 0.5f) * (float)moteur->width);
      *screen_y = (int)((1.0f - (ndc_point->y * 0.5f + 0.5f)) * (float)moteur->height);
}

static bool m3d_clip_line_to_near_plane(const Moteur3D* moteur, Vect4* a_view, Vect4* b_view){
      const float near_z = moteur->znear;
      const bool a_inside = a_view->z >= near_z;
      const bool b_inside = b_view->z >= near_z;

      if(a_inside && b_inside){
            return true;
      }

      if(!a_inside && !b_inside){
            return false;
      }

      {
            float dz = b_view->z - a_view->z;
            if(fabsf(dz) < 0.00001f){
                  return false;
            }

            {
                  float t = (near_z - a_view->z) / dz;
                  Vect4 p = {
                        a_view->x + (b_view->x - a_view->x) * t,
                        a_view->y + (b_view->y - a_view->y) * t,
                        near_z,
                        1.0f
                  };

                  if(!a_inside){
                        *a_view = p;
                  }else{
                        *b_view = p;
                  }
            }
      }

      return true;
}

static bool m3d_project_clipped_line(const Moteur3D* moteur, const Vect3* a, const Vect3* b, M3D_PointProjection* pa, M3D_PointProjection* pb){
      Vect4 a_view = m3d_world_to_view(moteur, a);
      Vect4 b_view = m3d_world_to_view(moteur, b);
      Vect4 a_clip;
      Vect4 b_clip;

      if(!m3d_clip_line_to_near_plane(moteur, &a_view, &b_view)){
            return false;
      }

      a_clip = m3d_view_to_clip(moteur, &a_view);
      b_clip = m3d_view_to_clip(moteur, &b_view);

      if(!m3d_clip_to_ndc(&a_clip, &pa->ndc) || !m3d_clip_to_ndc(&b_clip, &pb->ndc)){
            return false;
      }

      m3d_ndc_to_screen(moteur, &pa->ndc, &pa->screen_x, &pa->screen_y);
      m3d_ndc_to_screen(moteur, &pb->ndc, &pb->screen_x, &pb->screen_y);
      pa->depth = a_view.z;
      pb->depth = b_view.z;

      pa->visible = m3d_is_ndc_visible(&pa->ndc);
      pb->visible = m3d_is_ndc_visible(&pb->ndc);
      return true;
}

static Vect4 m3d_intersect_near_plane(const Vect4* a, const Vect4* b, float near_z){
      float t = (near_z - a->z) / (b->z - a->z);
      return (Vect4){
            a->x + (b->x - a->x) * t,
            a->y + (b->y - a->y) * t,
            near_z,
            1.0f
      };
}

static int m3d_clip_triangle_to_near_plane(const Moteur3D* moteur, const Vect4* a, const Vect4* b, const Vect4* c, Vect4 out_vertices[4]){
      Vect4 in_vertices[4];
      Vect4 temp_vertices[4];
      int in_count = 3;
      int i;
      float near_z = moteur->znear;

      in_vertices[0] = *a;
      in_vertices[1] = *b;
      in_vertices[2] = *c;

      {
            int out_count = 0;
            for(i = 0; i < in_count; i++){
                  const Vect4* current = &in_vertices[i];
                  const Vect4* next = &in_vertices[(i + 1) % in_count];
                  bool current_inside = current->z >= near_z;
                  bool next_inside = next->z >= near_z;

                  if(current_inside && next_inside){
                        temp_vertices[out_count++] = *next;
                  }else if(current_inside && !next_inside){
                        if(fabsf(next->z - current->z) > 0.00001f){
                              temp_vertices[out_count++] = m3d_intersect_near_plane(current, next, near_z);
                        }
                  }else if(!current_inside && next_inside){
                        if(fabsf(next->z - current->z) > 0.00001f){
                              temp_vertices[out_count++] = m3d_intersect_near_plane(current, next, near_z);
                        }
                        temp_vertices[out_count++] = *next;
                  }
            }

            in_count = out_count;
      }

      if(in_count < 3){
            return 0;
      }

      for(i = 0; i < in_count; i++){
            out_vertices[i] = temp_vertices[i];
      }

      return in_count;
}

static bool m3d_project_view_vertex(const Moteur3D* moteur, const Vect4* view_point, M3D_PointProjection* projection){
      Vect4 clip = m3d_view_to_clip(moteur, view_point);

      if(!m3d_clip_to_ndc(&clip, &projection->ndc)){
            return false;
      }

      m3d_ndc_to_screen(moteur, &projection->ndc, &projection->screen_x, &projection->screen_y);
      projection->depth = view_point->z;
      projection->visible = m3d_is_ndc_visible(&projection->ndc);
      return true;
}

static bool m3d_triangle_outside_xy_frustum(const Moteur3D* moteur, const Vect4* v0, const Vect4* v1, const Vect4* v2){
      Vect4 c0 = m3d_view_to_clip(moteur, v0);
      Vect4 c1 = m3d_view_to_clip(moteur, v1);
      Vect4 c2 = m3d_view_to_clip(moteur, v2);

      if((c0.x < -c0.w && c1.x < -c1.w && c2.x < -c2.w)
      || (c0.x > c0.w && c1.x > c1.w && c2.x > c2.w)
      || (c0.y < -c0.w && c1.y < -c1.w && c2.y < -c2.w)
      || (c0.y > c0.w && c1.y > c1.w && c2.y > c2.w)){
            return true;
      }

      return false;
}

//MARK: Internal Draw Helpers

static inline void m3d_plot_pixel_depth(M3D_Engine* engine, int x, int y, float depth, uint32_t color){
      if(x >= 0 && x < engine->camera.width && y >= 0 && y < engine->camera.height){
            size_t index = (size_t)y * (size_t)engine->camera.width + (size_t)x;
            if(depth < engine->zbuffer[index]){
                  engine->zbuffer[index] = depth;
                  engine->buffer[index] = color;
            }
      }
}

static bool m3d_draw_line_screen_depth(M3D_Engine* engine, int x0, int y0, float z0, int x1, int y1, float z1, uint32_t color){
      int dx = abs(x1 - x0);
      int sx = x0 < x1 ? 1 : -1;
      int dy = -abs(y1 - y0);
      int sy = y0 < y1 ? 1 : -1;
      int err = dx + dy;
      int step = 0;
      int steps = dx > -dy ? dx : -dy;

      if(steps <= 0){
            steps = 1;
      }

      while(1){
            float t = (float)step / (float)steps;
            float depth = z0 + (z1 - z0) * t;
            m3d_plot_pixel_depth(engine, x0, y0, depth, color);

            if(x0 == x1 && y0 == y1){
                  break;
            }

            {
                  int e2 = 2 * err;
                  if(e2 >= dy){
                        err += dy;
                        x0 += sx;
                  }
                  if(e2 <= dx){
                        err += dx;
                        y0 += sy;
                  }
            }

            step++;
      }

      return true;
}

static void m3d_plot_thick_pixel_depth(M3D_Engine* engine, int x, int y, float depth, uint32_t color, int thickness){
      int half_min;
      int half_max;
      int oy;

      if(thickness <= 1){
            m3d_plot_pixel_depth(engine, x, y, depth, color);
            return;
      }

      half_min = -(thickness / 2);
      half_max = thickness - 1 + half_min;

      for(oy = half_min; oy <= half_max; oy++){
            int ox;
            for(ox = half_min; ox <= half_max; ox++){
                  m3d_plot_pixel_depth(engine, x + ox, y + oy, depth, color);
            }
      }
}

static bool m3d_draw_line_screen_thick_depth(M3D_Engine* engine, int x0, int y0, float z0, int x1, int y1, float z1, uint32_t color, int thickness){
      int dx = abs(x1 - x0);
      int sx = x0 < x1 ? 1 : -1;
      int dy = -abs(y1 - y0);
      int sy = y0 < y1 ? 1 : -1;
      int err = dx + dy;
      int step = 0;
      int steps = dx > -dy ? dx : -dy;

      if(steps <= 0){
            steps = 1;
      }

      while(1){
            float t = (float)step / (float)steps;
            float depth = z0 + (z1 - z0) * t;
            m3d_plot_thick_pixel_depth(engine, x0, y0, depth, color, thickness);

            if(x0 == x1 && y0 == y1){
                  break;
            }

            {
                  int e2 = 2 * err;
                  if(e2 >= dy){
                        err += dy;
                        x0 += sx;
                  }
                  if(e2 <= dx){
                        err += dx;
                        y0 += sy;
                  }
            }

            step++;
      }

      return true;
}

static inline float m3d_edge_function(float ax, float ay, float bx, float by, float px, float py){
      return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}

static bool m3d_draw_triangle_screen(M3D_Engine* engine, int ax, int ay, float za, int bx, int by, float zb, int cx, int cy, float zc, uint32_t color){
      int min_x;
      int min_y;
      int max_x;
      int max_y;
      int x;
      int y;
      float area = m3d_edge_function((float)ax, (float)ay, (float)bx, (float)by, (float)cx, (float)cy);
      float inv_area;
      float orient;

      float e0_dx;
      float e0_dy;
      float e1_dx;
      float e1_dy;
      float e2_dx;
      float e2_dy;

      float depth_w0;
      float depth_w1;
      float depth_w2;

      float start_px;
      float start_py;
      float w0_row;
      float w1_row;
      float w2_row;

      if(fabsf(area) < 0.00001f){
            return false;
      }

      inv_area = 1.0f / area;
      orient = area > 0.0f ? 1.0f : -1.0f;

      e0_dx = (float)(cy - by);
      e0_dy = (float)(bx - cx);
      e1_dx = (float)(ay - cy);
      e1_dy = (float)(cx - ax);
      e2_dx = (float)(by - ay);
      e2_dy = (float)(ax - bx);

      depth_w0 = za * inv_area;
      depth_w1 = zb * inv_area;
      depth_w2 = zc * inv_area;

      min_x = ax;
      if(bx < min_x){
            min_x = bx;
      }
      if(cx < min_x){
            min_x = cx;
      }

      min_y = ay;
      if(by < min_y){
            min_y = by;
      }
      if(cy < min_y){
            min_y = cy;
      }

      max_x = ax;
      if(bx > max_x){
            max_x = bx;
      }
      if(cx > max_x){
            max_x = cx;
      }

      max_y = ay;
      if(by > max_y){
            max_y = by;
      }
      if(cy > max_y){
            max_y = cy;
      }

      if(min_x < 0){
            min_x = 0;
      }
      if(min_y < 0){
            min_y = 0;
      }
      if(max_x >= engine->camera.width){
            max_x = engine->camera.width - 1;
      }
      if(max_y >= engine->camera.height){
            max_y = engine->camera.height - 1;
      }

      if(min_x > max_x || min_y > max_y){
            return false;
      }

      start_px = (float)min_x + 0.5f;
      start_py = (float)min_y + 0.5f;

      w0_row = m3d_edge_function((float)bx, (float)by, (float)cx, (float)cy, start_px, start_py);
      w1_row = m3d_edge_function((float)cx, (float)cy, (float)ax, (float)ay, start_px, start_py);
      w2_row = m3d_edge_function((float)ax, (float)ay, (float)bx, (float)by, start_px, start_py);

      for(y = min_y; y <= max_y; y++){
            float w0 = w0_row;
            float w1 = w1_row;
            float w2 = w2_row;

            for(x = min_x; x <= max_x; x++){
                  if((w0 * orient) >= 0.0f && (w1 * orient) >= 0.0f && (w2 * orient) >= 0.0f){
                        float depth = w0 * depth_w0 + w1 * depth_w1 + w2 * depth_w2;
                        m3d_plot_pixel_depth(engine, x, y, depth, color);
                  }

                  w0 += e0_dx;
                  w1 += e1_dx;
                  w2 += e2_dx;
            }

            w0_row += e0_dy;
            w1_row += e1_dy;
            w2_row += e2_dy;
      }

      return true;
}

//MARK: Drawing API

bool M3D_draw_projected_point(M3D_Engine* engine, const M3D_PointProjection* projection, uint32_t color){
      if(!projection->visible){
            return false;
      }

      if(projection->screen_x < 0 || projection->screen_x >= engine->camera.width
      || projection->screen_y < 0 || projection->screen_y >= engine->camera.height){
            return false;
      }

      m3d_plot_pixel_depth(engine, projection->screen_x, projection->screen_y, projection->depth, color);
      return true;
}

M3D_PointProjection M3D_draw_point(M3D_Engine* engine, const Vect3* world_point, uint32_t color){
      M3D_PointProjection projection = M3D_project_point(&engine->camera, world_point);
      M3D_draw_projected_point(engine, &projection, color);
      return projection;
}

size_t M3D_draw_points(M3D_Engine* engine, const Vect3* world_points, size_t count, uint32_t color){
      size_t i;
      size_t drawn_count = 0;

      for(i = 0; i < count; i++){
            M3D_PointProjection projection = M3D_project_point(&engine->camera, &world_points[i]);
            if(M3D_draw_projected_point(engine, &projection, color)){
                  drawn_count++;
            }
      }

      return drawn_count;
}

bool M3D_draw_line(M3D_Engine* engine, const Vect3* a, const Vect3* b, uint32_t color){
      M3D_PointProjection pa;
      M3D_PointProjection pb;

      if(!m3d_project_clipped_line(&engine->camera, a, b, &pa, &pb)){
            return false;
      }

      if(!pa.visible && !pb.visible){
            return false;
      }

      return m3d_draw_line_screen_depth(engine, pa.screen_x, pa.screen_y, pa.depth, pb.screen_x, pb.screen_y, pb.depth, color);
}

size_t M3D_draw_lines(M3D_Engine* engine, const M3D_Line3D* lines, size_t count, uint32_t color){
      size_t i;
      size_t drawn_count = 0;

      for(i = 0; i < count; i++){
            if(M3D_draw_line(engine, &lines[i].a, &lines[i].b, color)){
                  drawn_count++;
            }
      }

      return drawn_count;
}

size_t M3D_draw_thick_lines(M3D_Engine* engine, const M3D_Line3D* lines, size_t count, uint32_t color, int thickness){
      size_t i;
      size_t drawn_count = 0;

      if(engine == NULL || lines == NULL || count == 0){
            return 0;
      }

      if(thickness <= 1){
            return M3D_draw_lines(engine, lines, count, color);
      }

      for(i = 0; i < count; i++){
            M3D_PointProjection pa;
            M3D_PointProjection pb;

            if(!m3d_project_clipped_line(&engine->camera, &lines[i].a, &lines[i].b, &pa, &pb)){
                  continue;
            }

            if(!pa.visible && !pb.visible){
                  continue;
            }

            if(m3d_draw_line_screen_thick_depth(engine, pa.screen_x, pa.screen_y, pa.depth, pb.screen_x, pb.screen_y, pb.depth, color, thickness)){
                  drawn_count++;
            }
      }

      return drawn_count;
}

bool M3D_draw_triangle(M3D_Engine* engine, const Vect3* a, const Vect3* b, const Vect3* c, uint32_t color){
      Vect4 a_view;
      Vect4 b_view;
      Vect4 c_view;
      Vect4 clipped[4];
      int clipped_count;
      int i;
      bool drawn = false;

      a_view = m3d_world_to_view(&engine->camera, a);
      b_view = m3d_world_to_view(&engine->camera, b);
      c_view = m3d_world_to_view(&engine->camera, c);

      clipped_count = m3d_clip_triangle_to_near_plane(&engine->camera, &a_view, &b_view, &c_view, clipped);
      if(clipped_count < 3){
            return false;
      }

      for(i = 1; i + 1 < clipped_count; i++){
            Vect4 v0 = clipped[0];
            Vect4 v1 = clipped[i];
            Vect4 v2 = clipped[i + 1];
            float abx = v1.x - v0.x;
            float aby = v1.y - v0.y;
            float acx = v2.x - v0.x;
            float acy = v2.y - v0.y;
            float normal_z = abx * acy - aby * acx;
            M3D_PointProjection p0;
            M3D_PointProjection p1;
            M3D_PointProjection p2;

            if(normal_z <= 0.0f){
                  continue;
            }

            if(m3d_triangle_outside_xy_frustum(&engine->camera, &v0, &v1, &v2)){
                  continue;
            }

            if(!m3d_project_view_vertex(&engine->camera, &v0, &p0)
            || !m3d_project_view_vertex(&engine->camera, &v1, &p1)
            || !m3d_project_view_vertex(&engine->camera, &v2, &p2)){
                  continue;
            }

            if(m3d_draw_triangle_screen(
                  engine,
                  p0.screen_x,
                  p0.screen_y,
                  p0.depth,
                  p1.screen_x,
                  p1.screen_y,
                  p1.depth,
                  p2.screen_x,
                  p2.screen_y,
                  p2.depth,
                  color
            )){
                  drawn = true;
            }
      }

      return drawn;
}

size_t M3D_draw_triangles(M3D_Engine* engine, const M3D_Triangle3D* triangles, size_t count, uint32_t color){
      size_t i;
      size_t drawn_count = 0;

      if(engine == NULL || triangles == NULL || count == 0){
            return 0;
      }

      for(i = 0; i < count; i++){
            if(M3D_draw_triangle(engine, &triangles[i].a, &triangles[i].b, &triangles[i].c, color)){
                  drawn_count++;
            }
      }

      return drawn_count;
}

//MARK: Camera Utility API

Vect3 M3D_Convert_Vect(Moteur3D* moteur, Vect3* vect){
      return M3D_project_point(moteur, vect).ndc;
}

M3D_PointProjection M3D_project_point(const Moteur3D* moteur, const Vect3* world_point){
      Vect4 view;
      Vect4 clip;
      M3D_PointProjection projection;

      projection.visible = false;
      projection.depth = INFINITY;
      projection.screen_x = 0;
      projection.screen_y = 0;

      view = m3d_world_to_view(moteur, world_point);
      clip = m3d_view_to_clip(moteur, &view);

      if(clip.w > 0.0f && m3d_clip_to_ndc(&clip, &projection.ndc)){
            m3d_ndc_to_screen(moteur, &projection.ndc, &projection.screen_x, &projection.screen_y);
            projection.depth = view.z;
            projection.visible = m3d_is_ndc_visible(&projection.ndc);
      }else{
            projection.ndc = (Vect3){0.0f, 0.0f, 0.0f};
      }

      return projection;
}
