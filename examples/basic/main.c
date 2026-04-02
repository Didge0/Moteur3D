#include <stdio.h>
#define ADVANCED_API
#include "Moteur3D/Moteur3D.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#if defined(_MSC_VER) && defined(_DEBUG)
#include <crtdbg.h>
#endif

#define WIDTH 1200
#define HEIGHT 900
#define DEBUG_TEXT_SCALE 2.0f

static void update_auto_orbit_test(M3D_Engine* engine, bool enabled, float delta_seconds){
      if(enabled && engine->camera.mode == CAM_MODE_ORBIT){
            //move_camera_backward(&engine->camera, 0.2f);
            //rotate_camera_yaw(&engine->camera, 45.0f * delta_seconds);
            //rotate_camera_pitch(&engine->camera, 20.0f * delta_seconds * sinf(SDL_GetTicks() / 500.0f));
      }
}

int main(){
      M3D_Engine engine;
      Moteur3D_InitData initData;
      M3D_fill_default_init_data(&initData, WIDTH, HEIGHT);

      /* Exemple mode custom: change quelques valeurs puis initialise le moteur. */
      initData.mode = CAM_MODE_ORBIT;
      initData.target = (Vect3){0.0f, 0.0f, 0.0f};
      initData.distance = 4.0f;

      Vect3 cube_vertices[8] = {
            (Vect3){-1,-1,-1},
            (Vect3){ 1,-1,-1},
            (Vect3){ 1, 1,-1},
            (Vect3){-1, 1,-1},
            (Vect3){-1,-1, 1},
            (Vect3){ 1,-1, 1},
            (Vect3){ 1, 1, 1},
            (Vect3){-1, 1, 1}
      };

      M3D_Line3D cube_lines[12] = {
            (M3D_Line3D){cube_vertices[0], cube_vertices[1]},
            (M3D_Line3D){cube_vertices[1], cube_vertices[2]},
            (M3D_Line3D){cube_vertices[2], cube_vertices[3]},
            (M3D_Line3D){cube_vertices[3], cube_vertices[0]},
            (M3D_Line3D){cube_vertices[4], cube_vertices[5]},
            (M3D_Line3D){cube_vertices[5], cube_vertices[6]},
            (M3D_Line3D){cube_vertices[6], cube_vertices[7]},
            (M3D_Line3D){cube_vertices[7], cube_vertices[4]},
            (M3D_Line3D){cube_vertices[0], cube_vertices[4]},
            (M3D_Line3D){cube_vertices[1], cube_vertices[5]},
            (M3D_Line3D){cube_vertices[2], cube_vertices[6]},
            (M3D_Line3D){cube_vertices[3], cube_vertices[7]}
      };

      M3D_Triangle3D cube_triangles[12] = {
            (M3D_Triangle3D){cube_vertices[0], cube_vertices[1], cube_vertices[2]},
            (M3D_Triangle3D){cube_vertices[0], cube_vertices[2], cube_vertices[3]},
            (M3D_Triangle3D){cube_vertices[4], cube_vertices[6], cube_vertices[5]},
            (M3D_Triangle3D){cube_vertices[4], cube_vertices[7], cube_vertices[6]},
            (M3D_Triangle3D){cube_vertices[0], cube_vertices[4], cube_vertices[5]},
            (M3D_Triangle3D){cube_vertices[0], cube_vertices[5], cube_vertices[1]},
            (M3D_Triangle3D){cube_vertices[3], cube_vertices[2], cube_vertices[6]},
            (M3D_Triangle3D){cube_vertices[3], cube_vertices[6], cube_vertices[7]},
            (M3D_Triangle3D){cube_vertices[0], cube_vertices[3], cube_vertices[7]},
            (M3D_Triangle3D){cube_vertices[0], cube_vertices[7], cube_vertices[4]},
            (M3D_Triangle3D){cube_vertices[1], cube_vertices[5], cube_vertices[6]},
            (M3D_Triangle3D){cube_vertices[1], cube_vertices[6], cube_vertices[2]}
      };

      bool is_opened = true;
      if(!M3D_init_custom(&engine, &initData, "Moving Masse")){
            return false;
      }

      M3D_InputState input = {0};
      bool auto_orbit_test_enabled = false;
      Uint64 prev_ticks = SDL_GetTicks();
      float smoothed_hz = 0.0f;

      char data[1024];
      char hz_text[64];
      SDL_FRect dataRect = (SDL_FRect){0,0,(float)engine.camera.width,40};

      while(is_opened){
            Uint64 now_ticks = SDL_GetTicks();
            float delta_seconds = (float)(now_ticks - prev_ticks) / 1000.0f;
            SDL_Event event;
            Vect3 pt1_ndc = (Vect3){0};
            int pt1_screen_x = 0;
            int pt1_screen_y = 0;
            int pt1_visible = 0;

            prev_ticks = now_ticks;
            if(delta_seconds < 0.0f){
                  delta_seconds = 0.0f;
            }
            if(delta_seconds > 0.100f){
                  delta_seconds = 0.100f;
            }

            if(delta_seconds > 0.00001f){
                  float instant_hz = 1.0f / delta_seconds;
                  if(smoothed_hz <= 0.0f){
                        smoothed_hz = instant_hz;
                  }else{
                        smoothed_hz = smoothed_hz * 0.90f + instant_hz * 0.10f;
                  }
            }

            while(SDL_PollEvent(&event)){
                  if (event.type == SDL_EVENT_QUIT) {
                        is_opened = false;
                  }else if( event.type == SDL_EVENT_KEY_DOWN){
                        if(event.key.key == SDLK_T && !event.key.repeat){
                              auto_orbit_test_enabled = !auto_orbit_test_enabled;
                        }

                        int should_quit = 0;
                        M3D_bind_default_key_down(&engine.camera, &input, (int)event.key.key, event.key.repeat ? 1 : 0, &should_quit);
                        if(should_quit){
                              is_opened = false;
                        }
                  }else if(event.type == SDL_EVENT_KEY_UP){
                        M3D_bind_default_key_up(&input, (int)event.key.key);
                  }else if(event.type == SDL_EVENT_MOUSE_MOTION){
                        M3D_bind_default_mouse_motion(&input, (float)event.motion.xrel, (float)event.motion.yrel);
                  }else if(event.type == SDL_EVENT_MOUSE_WHEEL){
                        M3D_bind_default_mouse_wheel(&engine.camera, event.wheel.y);
                  }
            }

            M3D_apply_input_state(&engine.camera, &input, delta_seconds);
            update_auto_orbit_test(&engine, auto_orbit_test_enabled, delta_seconds);

            M3D_clear_frame(&engine, 0xa9cf6dFF);

            M3D_draw_triangles(&engine, cube_triangles, 12, 0x7EC4A6FF);
            M3D_draw_thick_lines(&engine, cube_lines, 12, 0x936DCFFF, 3);


            snprintf(
                  data,
                  sizeof(data),
                  "mode=%s Rot=%s cam=(%.2f,%.2f,%.2f) yaw=%.1f pitch=%.1f",
                  engine.camera.mode == CAM_MODE_ORBIT ? "ORBIT" : "FPS",
                  auto_orbit_test_enabled ? "ON" : "OFF",
                  engine.camera.pos.x,
                  engine.camera.pos.y,
                  engine.camera.pos.z,
                  engine.camera.yaw * (180.0f / (float)M_PI),
                  engine.camera.pitch * (180.0f / (float)M_PI)
            );

            M3D_end_frame(&engine);

            SDL_SetRenderDrawColor(engine.renderer, 255,255,255,255);
            SDL_RenderFillRect(engine.renderer, &dataRect);

            SDL_SetRenderDrawColor(engine.renderer, 50,150,50,255);
            SDL_SetRenderScale(engine.renderer, DEBUG_TEXT_SCALE, DEBUG_TEXT_SCALE);
            SDL_RenderDebugTextFormat(engine.renderer, 5, 5, "%s", data);

            snprintf(hz_text, sizeof(hz_text), "Hz: %.1f", smoothed_hz);
            {
                  int logical_width = (int)((float)engine.camera.width / DEBUG_TEXT_SCALE);
                  int text_x = logical_width - 5 - (int)strlen(hz_text) * 8;
                  if(text_x < 5){
                        text_x = 5;
                  }
                  SDL_RenderDebugTextFormat(engine.renderer, text_x, 5, "%s", hz_text);
            }

            SDL_SetRenderScale(engine.renderer, 1.0f, 1.0f);
            M3D_present_frame(&engine);
      }

      M3D_shutdown(&engine);

#if defined(_MSC_VER) && defined(_DEBUG)
      if(_CrtDumpMemoryLeaks()){
            printf("Fuites memoire !\n");
      }
#endif


      return 0;
}