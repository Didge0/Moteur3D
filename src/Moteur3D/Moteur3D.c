#include "Moteur3D/Moteur3D.h"
#include "Moteur3D_internal.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include <stdlib.h>
#define DEG_TO_RAD(x)  ((x*M_PI)/180)

#define M3D_KEYBOARD_ROTATION_SPEED_DEG_PER_SEC 300.0f
#define M3D_MOUSE_SENSITIVITY_DEG 0.15f
#define M3D_MOUSE_WHEEL_ZOOM_STEP 0.25f
#define M3D_FPS_MOVE_SPEED_UNITS_PER_SEC 12.0f

/* Forward declarations for functions used before their definition. */
void rotate_camera_yaw(Moteur3D* moteur, float delta_deg);
void rotate_camera_pitch(Moteur3D* moteur, float delta_deg);
void move_camera_right(Moteur3D* moteur, float speed);
void move_camera_left(Moteur3D* moteur, float speed);
void move_camera_up(Moteur3D* moteur, float speed);
void move_camera_down(Moteur3D* moteur, float speed);
void move_camera_forward(Moteur3D* moteur, float speed);
void move_camera_backward(Moteur3D* moteur, float speed);

//MARK: Internal Math Helpers

static inline void add_float(void* Dst, const void* val1, const void* val2){ *(float*)Dst = *(float*)val1 + *(float*)val2; }
static inline void mul_float(void* Dst, const void* val1, const void* val2){ *(float*)Dst = *(float*)val1 * (*(float*)val2); }

static inline void normalize(Vect3* vect){
      float norm = sqrtf(vect->x*vect->x + vect->y*vect->y + vect->z*vect->z);
      if(norm > 0.00001f){
            vect->x /= norm;
            vect->y /= norm;
            vect->z /= norm;
      }
}

static inline Vect3 prod_vect(const Vect3* v1, const Vect3* v2){
      return (Vect3){
            (v1->y*v2->z) - (v1->z*v2->y),
            (v1->z*v2->x) - (v1->x*v2->z),
            (v1->x*v2->y) - (v1->y*v2->x)
      };
}

static inline float vect3_prod_scal(Vect3* v1, Vect3* v2){
      return v1->x*v2->x + v1->y*v2->y + v1->z*v2->z;
}

static inline float vect3_distance(const Vect3* a, const Vect3* b){
      float dx = a->x - b->x;
      float dy = a->y - b->y;
      float dz = a->z - b->z;
      return sqrtf(dx*dx + dy*dy + dz*dz);
}

static inline float clamp_float(float value, float min_value, float max_value){
      if(value < min_value){
            return min_value;
      }
      if(value > max_value){
            return max_value;
      }
      return value;
}

static inline M3D_CameraInternal* m3d_cam_internal(Moteur3D* moteur){
      return moteur->internal;
}

static inline bool m3d_translate_camera_fps(Moteur3D* moteur, const Vect3* axis, float speed){
      if(moteur->mode != CAM_MODE_FPS){
            return false;
      }

      moteur->pos.x += axis->x * speed;
      moteur->pos.y += axis->y * speed;
      moteur->pos.z += axis->z * speed;
      return true;
}

//MARK: Internal Camera Helpers

static inline void update_matrice(Moteur3D* moteur){
      M3D_CameraInternal* cami = m3d_cam_internal(moteur);
      float view_tab[16]=
      {
                          moteur->right.x,   moteur->right.y,   moteur->right.z,  -vect3_prod_scal(&moteur->right, &moteur->pos),
                                moteur->up.x,      moteur->up.y,      moteur->up.z,     -vect3_prod_scal(&moteur->up, &moteur->pos),
                        moteur->forward.x, moteur->forward.y, moteur->forward.z, -vect3_prod_scal(&moteur->forward, &moteur->pos),
                            0,                 0,                 0,                    1
      };
      memcpy(cami->view.data, view_tab, sizeof(float) * 16);

      {
            float tanfov = 1 / (tanf(DEG_TO_RAD(moteur->fov) * 0.5f));
            float proj_tab[16] = {
                  tanfov / moteur->aspect, 0, 0, 0,
                  0, tanfov, 0, 0,
                  0, 0, (moteur->zfar + moteur->znear) / (moteur->znear - moteur->zfar), (2 * moteur->zfar * moteur->znear) / (moteur->znear - moteur->zfar),
                  0, 0, 1, 0
            };
            memcpy(cami->proj.data, proj_tab, sizeof(float) * 16);
      }

      matrice_prod(&cami->viewProj, &cami->proj, &cami->view, add_float, mul_float);
}

static inline void M3D_update_camera_pose(Moteur3D* moteur){
      const Vect3 world_up = {0, 0, 1};

      moteur->forward = (Vect3){
            cosf(moteur->pitch) * cosf(moteur->yaw),
            cosf(moteur->pitch) * sinf(moteur->yaw),
            sinf(moteur->pitch)
      };
      normalize(&moteur->forward);

      moteur->right = prod_vect(&world_up, &moteur->forward);
      normalize(&moteur->right);

      moteur->up = prod_vect(&moteur->forward, &moteur->right);
      normalize(&moteur->up);

      if(moteur->mode == CAM_MODE_ORBIT){
            if(moteur->distance < 0.001f){
                  moteur->distance = 0.001f;
            }
            moteur->pos.x = moteur->target.x - moteur->forward.x * moteur->distance;
            moteur->pos.y = moteur->target.y - moteur->forward.y * moteur->distance;
            moteur->pos.z = moteur->target.z - moteur->forward.z * moteur->distance;
      }

      moteur->aspect = ((float)moteur->width) / ((float)moteur->height);
      update_matrice(moteur);
}

//MARK: Camera Initialization API

void M3D_initCamera(Moteur3D* moteur, const Moteur3D_InitData* data){
      M3D_CameraInternal* cami;
      moteur->mode = data->mode;

      moteur->pos = data->pos;
      moteur->yaw = DEG_TO_RAD(data->yaw);
      moteur->pitch = DEG_TO_RAD(data->pitch);
      moteur->roll = DEG_TO_RAD(data->roll);

      moteur->target = data->target;
      moteur->distance = data->distance;
      if(moteur->distance <= 0.0f){
            moteur->distance = vect3_distance(&moteur->pos, &moteur->target);
            if(moteur->distance <= 0.0f){
                  moteur->distance = 5.0f;
            }
      }

      moteur->fov = data->fov;
      moteur->zfar = data->zfar;
      moteur->znear = data->znear;

      moteur->width = data->width;
      moteur->height = data->height;

      cami = (M3D_CameraInternal*)malloc(sizeof(M3D_CameraInternal));
      moteur->internal = cami;
      if(cami == NULL){
            return;
      }

      cami->view = matrice_init(4, 4, sizeof(float));
      cami->proj = matrice_init(4, 4, sizeof(float));
      cami->viewProj = matrice_init(4, 4, sizeof(float));

      M3D_update_camera_pose(moteur);
}

void M3D_fill_default_init_data(Moteur3D_InitData* data, int width, int height){
      data->mode = CAM_MODE_ORBIT;
      data->pos = (Vect3){-4.0f, 0.0f, 0.0f};
      data->yaw = 0.0f;
      data->pitch = 0.0f;
      data->roll = 0.0f;
      data->target = (Vect3){0.0f, 0.0f, 0.0f};
      data->distance = 4.0f;
      data->width = width;
      data->height = height;
      data->fov = 60.0f;
      data->znear = 1.0f;
      data->zfar = 100.0f;
}

//MARK: Engine Lifecycle API

bool M3D_init_sdl(M3D_Engine* engine, int width, int height, const char* window_title){
      int render_width = width;
      int render_height = height;

      if(!SDL_Init(SDL_INIT_VIDEO)){
            return false;
      }

      if(!SDL_CreateWindowAndRenderer(window_title != NULL ? window_title : "Moteur3D", width, height, 0, &engine->window, &engine->renderer)){
            SDL_Quit();
            return false;
      }

      SDL_GetWindowSize(engine->window, &render_width, &render_height);
      engine->camera.width = render_width;
      engine->camera.height = render_height;

      engine->texture = SDL_CreateTexture(engine->renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, render_width, render_height);
      if(engine->texture == NULL){
            SDL_DestroyRenderer(engine->renderer);
            SDL_DestroyWindow(engine->window);
            engine->renderer = NULL;
            engine->window = NULL;
            SDL_Quit();
            return false;
      }

      engine->buffer = (uint32_t*)malloc((size_t)render_width * (size_t)render_height * sizeof(uint32_t));
      if(engine->buffer == NULL){
            SDL_DestroyTexture(engine->texture);
            SDL_DestroyRenderer(engine->renderer);
            SDL_DestroyWindow(engine->window);
            engine->texture = NULL;
            engine->renderer = NULL;
            engine->window = NULL;
            SDL_Quit();
            return false;
      }

      engine->zbuffer = (float*)malloc((size_t)render_width * (size_t)render_height * sizeof(float));
      if(engine->zbuffer == NULL){
            free(engine->buffer);
            SDL_DestroyTexture(engine->texture);
            SDL_DestroyRenderer(engine->renderer);
            SDL_DestroyWindow(engine->window);
            engine->buffer = NULL;
            engine->texture = NULL;
            engine->renderer = NULL;
            engine->window = NULL;
            SDL_Quit();
            return false;
      }

      SDL_SetRenderTarget(engine->renderer, engine->texture);
      SDL_SetWindowRelativeMouseMode(engine->window, true);
      return true;
}

bool M3D_init_default(M3D_Engine* engine, int width, int height){
      Moteur3D_InitData initData;
      M3D_fill_default_init_data(&initData, width, height);
      return M3D_init_custom(engine, &initData, "Moteur3D");
}

bool M3D_init_custom(M3D_Engine* engine, const Moteur3D_InitData* initData, const char* window_title){
      memset(engine, 0, sizeof(*engine));

      M3D_initCamera(&engine->camera, initData);
      if(!M3D_init_sdl(engine, initData->width, initData->height, window_title)){
            M3D_end(&engine->camera);
            return false;
      }

      rotate_camera_yaw(&engine->camera, 0.0f);
      return true;
}

void M3D_shutdown(M3D_Engine* engine){
      M3D_end(&engine->camera);

      if(engine->zbuffer != NULL){
            free(engine->zbuffer);
            engine->zbuffer = NULL;
      }

      if(engine->buffer != NULL){
            free(engine->buffer);
            engine->buffer = NULL;
      }

      if(engine->window != NULL){
            SDL_SetWindowRelativeMouseMode(engine->window, false);
      }
      if(engine->texture != NULL){
            SDL_DestroyTexture(engine->texture);
            engine->texture = NULL;
      }
      if(engine->renderer != NULL){
            SDL_DestroyRenderer(engine->renderer);
            engine->renderer = NULL;
      }
      if(engine->window != NULL){
            SDL_DestroyWindow(engine->window);
            engine->window = NULL;
      }
      SDL_Quit();
}

//MARK: Frame API

void M3D_clear_frame(M3D_Engine* engine, uint32_t clear_color){
      int x;
      int y;

      for(y = 0; y < engine->camera.height; y++){
            for(x = 0; x < engine->camera.width; x++){
                  engine->buffer[y * engine->camera.width + x] = clear_color;
                  engine->zbuffer[y * engine->camera.width + x] = INFINITY;
            }
      }
}

void M3D_end_frame(M3D_Engine* engine){
      SDL_UpdateTexture(engine->texture, NULL, engine->buffer, engine->camera.width * (int)sizeof(uint32_t));

      SDL_SetRenderTarget(engine->renderer, NULL);
      SDL_RenderClear(engine->renderer);
      SDL_RenderTexture(engine->renderer, engine->texture, NULL, NULL);
}

void M3D_present_frame(M3D_Engine* engine){
      (void)engine;
      SDL_RenderPresent(engine->renderer);
}

//MARK: Camera Utility API

void M3D_end(Moteur3D* moteur){
      M3D_CameraInternal* cami = m3d_cam_internal(moteur);
      if(cami != NULL){
            matrice_free(&cami->view);
            matrice_free(&cami->proj);
            matrice_free(&cami->viewProj);
            free(cami);
            moteur->internal = NULL;
      }
}

//MARK: Camera Movement API

void move_camera_right(Moteur3D* moteur, float speed){
      if(m3d_translate_camera_fps(moteur, &moteur->right, speed)){
            update_matrice(moteur);
      }
}

void move_camera_left(Moteur3D* moteur, float speed){
      Vect3 left = {-moteur->right.x, -moteur->right.y, -moteur->right.z};
      if(m3d_translate_camera_fps(moteur, &left, speed)){
            update_matrice(moteur);
      }
}

void move_camera_up(Moteur3D* moteur, float speed){
      if(m3d_translate_camera_fps(moteur, &moteur->up, speed)){
            update_matrice(moteur);
      }
}

void move_camera_down(Moteur3D* moteur, float speed){
      Vect3 down = {-moteur->up.x, -moteur->up.y, -moteur->up.z};
      if(m3d_translate_camera_fps(moteur, &down, speed)){
            update_matrice(moteur);
      }
}

void move_camera_forward(Moteur3D* moteur, float speed){
      if(m3d_translate_camera_fps(moteur, &moteur->forward, speed)){
            update_matrice(moteur);
      }
}

void move_camera_backward(Moteur3D* moteur, float speed){
      Vect3 backward = {-moteur->forward.x, -moteur->forward.y, -moteur->forward.z};
      if(m3d_translate_camera_fps(moteur, &backward, speed)){
            update_matrice(moteur);
      }
}

void rotate_camera_yaw(Moteur3D* moteur, float delta_deg){
      const float full_turn = 2.0f * (float)M_PI;

      if(moteur->mode == CAM_MODE_FPS || moteur->mode == CAM_MODE_ORBIT){
            moteur->yaw += DEG_TO_RAD(delta_deg);
            moteur->yaw = fmodf(moteur->yaw, full_turn);
            if(moteur->yaw < 0.0f){
                  moteur->yaw += full_turn;
            }
            M3D_update_camera_pose(moteur);
      }
}

void rotate_camera_pitch(Moteur3D* moteur, float delta_deg){
      const float pitch_limit = DEG_TO_RAD(89.0f);

      if(moteur->mode == CAM_MODE_FPS || moteur->mode == CAM_MODE_ORBIT){
            moteur->pitch += DEG_TO_RAD(delta_deg);
            moteur->pitch = clamp_float(moteur->pitch, -pitch_limit, pitch_limit);
            M3D_update_camera_pose(moteur);
      }
}

//MARK: Camera Mode API

void M3D_zoom_orbit(Moteur3D* moteur, float delta){
      if(moteur->mode != CAM_MODE_ORBIT){
            return;
      }

      moteur->distance += delta;
      moteur->distance = clamp_float(moteur->distance, 0.001f, INFINITY);
      M3D_update_camera_pose(moteur);
}

void M3D_set_mode(Moteur3D* moteur, CameraMode mode){
      if(moteur->mode == mode){
            return;
      }

      if(mode == CAM_MODE_ORBIT){
            float new_distance = vect3_distance(&moteur->pos, &moteur->target);
            if(new_distance > 0.001f){
                  moteur->distance = new_distance;
            }else if(moteur->distance < 0.001f){
                  moteur->distance = 0.001f;
            }
      }

      moteur->mode = mode;
      M3D_update_camera_pose(moteur);
}

//MARK: Input Binding API

void M3D_bind_default_key_down(Moteur3D* moteur, M3D_InputState* input, int keycode, int is_repeat, int* should_quit){
      if(keycode == SDLK_ESCAPE){
            if(should_quit != NULL){
                  *should_quit = 1;
            }
            return;
      }

      if(keycode == SDLK_M && !is_repeat){
            if(moteur->mode == CAM_MODE_FPS){
                  M3D_set_mode(moteur, CAM_MODE_ORBIT);
            }else{
                  M3D_set_mode(moteur, CAM_MODE_FPS);
            }
            return;
      }

      if(keycode == SDLK_SPACE){
            input->move_up = 1;
      }else if(keycode == SDLK_TAB){
            input->move_down = 1;
      }else if(keycode == SDLK_D){
            input->move_right = 1;
      }else if(keycode == SDLK_Q){
            input->move_left = 1;
      }else if(keycode == SDLK_Z){
            input->move_forward = 1;
      }else if(keycode == SDLK_S){
            input->move_backward = 1;
      }else if(keycode == SDLK_LEFT){
            input->rot_left = 1;
      }else if(keycode == SDLK_RIGHT){
            input->rot_right = 1;
      }else if(keycode == SDLK_UP){
            input->rot_up = 1;
      }else if(keycode == SDLK_DOWN){
            input->rot_down = 1;
      }
}

void M3D_bind_default_key_up(M3D_InputState* input, int keycode){
      if(keycode == SDLK_SPACE){
            input->move_up = 0;
      }else if(keycode == SDLK_TAB){
            input->move_down = 0;
      }else if(keycode == SDLK_D){
            input->move_right = 0;
      }else if(keycode == SDLK_Q){
            input->move_left = 0;
      }else if(keycode == SDLK_Z){
            input->move_forward = 0;
      }else if(keycode == SDLK_S){
            input->move_backward = 0;
      }else if(keycode == SDLK_LEFT){
            input->rot_left = 0;
      }else if(keycode == SDLK_RIGHT){
            input->rot_right = 0;
      }else if(keycode == SDLK_UP){
            input->rot_up = 0;
      }else if(keycode == SDLK_DOWN){
            input->rot_down = 0;
      }
}

void M3D_bind_default_mouse_motion(M3D_InputState* input, float dx, float dy){
      input->mouse_dx += dx;
      input->mouse_dy += dy;
}

void M3D_bind_default_mouse_wheel(Moteur3D* moteur, float wheel_y){
      if(moteur->mode == CAM_MODE_ORBIT && wheel_y != 0.0f){
            M3D_zoom_orbit(moteur, -wheel_y * M3D_MOUSE_WHEEL_ZOOM_STEP);
      }
}

//MARK: Internal Input Helpers

static inline void m3d_apply_fps_moves(Moteur3D* moteur, const M3D_InputState* input, float move_delta){
      if(moteur->mode != CAM_MODE_FPS){
            return;
      }

      if(input->move_up){
            move_camera_up(moteur, move_delta);
      }
      if(input->move_down){
            move_camera_down(moteur, move_delta);
      }
      if(input->move_right){
            move_camera_right(moteur, move_delta);
      }
      if(input->move_left){
            move_camera_left(moteur, move_delta);
      }
      if(input->move_forward){
            move_camera_forward(moteur, move_delta);
      }
      if(input->move_backward){
            move_camera_backward(moteur, move_delta);
      }
}

static inline void m3d_apply_orbit_moves(Moteur3D* moteur, const M3D_InputState* input, float rot_delta_deg){
      if(input->move_left){
            rotate_camera_yaw(moteur, -rot_delta_deg);
      }
      if(input->move_right){
            rotate_camera_yaw(moteur, rot_delta_deg);
      }
      if(input->move_forward){
            rotate_camera_pitch(moteur, rot_delta_deg);
      }
      if(input->move_backward){
            rotate_camera_pitch(moteur, -rot_delta_deg);
      }
}

static inline void m3d_apply_common_rotations(Moteur3D* moteur, const M3D_InputState* input, float rot_delta_deg){
      if(input->rot_left){
            rotate_camera_yaw(moteur, -rot_delta_deg);
      }
      if(input->rot_right){
            rotate_camera_yaw(moteur, rot_delta_deg);
      }
      if(input->rot_up){
            rotate_camera_pitch(moteur, rot_delta_deg);
      }
      if(input->rot_down){
            rotate_camera_pitch(moteur, -rot_delta_deg);
      }
}

//MARK: Input Apply API

void M3D_apply_input_state(Moteur3D* moteur, M3D_InputState* input, float delta_seconds){
      float safe_dt = delta_seconds;
      float move_delta;
      float rot_delta_deg;

      if(safe_dt < 0.0f){
            safe_dt = 0.0f;
      }

      if(safe_dt > 0.100f){
            safe_dt = 0.100f;
      }

      move_delta = M3D_FPS_MOVE_SPEED_UNITS_PER_SEC * safe_dt;
      rot_delta_deg = M3D_KEYBOARD_ROTATION_SPEED_DEG_PER_SEC * safe_dt;

      if(moteur->mode == CAM_MODE_FPS){
            m3d_apply_fps_moves(moteur, input, move_delta);
      }else if(moteur->mode == CAM_MODE_ORBIT){
            m3d_apply_orbit_moves(moteur, input, rot_delta_deg);
      }

      m3d_apply_common_rotations(moteur, input, rot_delta_deg);

      if(input->mouse_dx != 0.0f){
            rotate_camera_yaw(moteur, input->mouse_dx * M3D_MOUSE_SENSITIVITY_DEG);
      }
      if(input->mouse_dy != 0.0f){
            rotate_camera_pitch(moteur, -input->mouse_dy * M3D_MOUSE_SENSITIVITY_DEG);
      }

      input->mouse_dx = 0.0f;
      input->mouse_dy = 0.0f;
}
