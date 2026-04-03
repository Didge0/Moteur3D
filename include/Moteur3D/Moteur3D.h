#ifndef MOTEUR3D_H
#define MOTEUR3D_H

#include "SDL3/SDL.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * API publique du moteur 3D.
 * Toutes les rotations envoyees a l'API sont en degres.
 * Les angles internes du moteur sont stockes en radians.
 */

//MARK: Camera Mode

typedef enum {
    CAM_MODE_ORBIT = 0,
    CAM_MODE_FPS   = 1
} CameraMode;

//MARK: Basic Types

typedef struct {
    float x, y, z;
} Vect3;

//MARK: Camera State

typedef struct M3D_CameraInternal M3D_CameraInternal;

typedef struct {
    /* Etat runtime principal de la camera */
    CameraMode mode;

    Vect3 pos;
    float yaw;    /* radians */
    float pitch;  /* radians */
    float roll;   /* radians */

    Vect3 forward;
    Vect3 right;
    Vect3 up;

    /* Taille de la vue */
    int width;
    int height;

    /* Parametres de projection */
    float fov;
    float aspect;
    float znear;
    float zfar;

    /* Parametres du mode orbit */
    Vect3 target;
    float distance;

    /* Etat interne (matrices/cache prive moteur) */
    M3D_CameraInternal* internal;
} Moteur3D;

//MARK: Init Data

typedef struct {
    /* Mode initial de la camera */
    CameraMode mode;

    /* Pose initiale */
    Vect3 pos;
    float yaw;    /* degres */
    float pitch;  /* degres */
    float roll;   /* degres */

    /* Utilises en mode orbit */
    Vect3 target;
    float distance;

    /* Taille du viewport */
    int width;
    int height;

    /* Parametres de projection */
    float fov;
    float znear;
    float zfar;
} Moteur3D_InitData;

//MARK: Input State

typedef struct {
    /* Deplacements */
    int move_forward;
    int move_backward;
    int move_left;
    int move_right;
    int move_up;
    int move_down;

    /* Rotations clavier */
    int rot_left;
    int rot_right;
    int rot_up;
    int rot_down;

    /* Delta souris accumule pendant la frame */
    float mouse_dx;
    float mouse_dy;
} M3D_InputState;

//MARK: Engine Context

typedef struct {
    Moteur3D camera;
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    uint32_t* buffer;
    float* zbuffer;
} M3D_Engine;

//MARK: Projection And Primitives

typedef struct {
    Vect3 ndc;
    float depth;
    int screen_x;
    int screen_y;
    bool visible;
} M3D_PointProjection;

typedef struct {
    Vect3 a;
    Vect3 b;
} M3D_Line3D;

typedef struct {
    Vect3 a;
    Vect3 b;
    Vect3 c;
} M3D_Triangle3D;

//MARK: Camera Initialization API

/* Initialise une camera et construit ses matrices de vue/projection. */
void M3D_initCamera(Moteur3D* moteur, const Moteur3D_InitData* data);

/* Remplit une config camera par defaut selon la taille donnee. */
void M3D_fill_default_init_data(Moteur3D_InitData* data, int width, int height);

//MARK: Engine Lifecycle API

/* Initialise SDL + fenetre + renderer + texture + buffer de rendu. */
bool M3D_init_sdl(M3D_Engine* engine, int width, int height, const char* window_title);

/* Initialise l'engine avec valeurs camera par defaut. */
bool M3D_init_default(M3D_Engine* engine, int width, int height);

/* Initialise l'engine avec valeurs camera personnalisees. */
bool M3D_init_custom(M3D_Engine* engine, const Moteur3D_InitData* initData, const char* window_title);

/* Libere tout le contexte engine (camera + SDL + buffer). */
void M3D_shutdown(M3D_Engine* engine);

//MARK: Frame API

/* Efface le buffer de frame avec une couleur RGBA8888. */
void M3D_clear_frame(M3D_Engine* engine, uint32_t clear_color);

/* Fin d'une frame: upload texture + rendu + present. */
void M3D_end_frame(M3D_Engine* engine);

/* Presente le rendu courant (a appeler apres les overlays debug). */
void M3D_present_frame(M3D_Engine* engine);

//MARK: Drawing API

/* Dessine un point deja projete; retourne true si dessine. */
bool M3D_draw_projected_point(M3D_Engine* engine, const M3D_PointProjection* projection, uint32_t color);

/* Projette puis dessine un point monde. */
M3D_PointProjection M3D_draw_point(M3D_Engine* engine, const Vect3* world_point, uint32_t color);

/* Dessine un lot de points monde et retourne le nombre dessine. */
size_t M3D_draw_points(M3D_Engine* engine, const Vect3* world_points, size_t count, uint32_t color);

/* Dessine une ligne 3D entre deux points monde. */
bool M3D_draw_line(M3D_Engine* engine, const Vect3* a, const Vect3* b, uint32_t color);

/* Dessine un lot de lignes 3D et retourne le nombre dessine. */
size_t M3D_draw_lines(M3D_Engine* engine, const M3D_Line3D* lines, size_t count, uint32_t color);

/* Dessine un lot de lignes 3D avec une épaisseur et retourne le nombre dessine. */
size_t M3D_draw_thick_lines(M3D_Engine* engine, const M3D_Line3D* lines, size_t count, uint32_t color, int thickness);

/* Dessine un triangle 3D rempli. */
bool M3D_draw_triangle(M3D_Engine* engine, const Vect3* a, const Vect3* b, const Vect3* c, uint32_t color);

/* Dessine un lot de triangles 3D remplis et retourne le nombre dessine. */
size_t M3D_draw_triangles(M3D_Engine* engine, const M3D_Triangle3D* triangles, size_t count, uint32_t color);

//MARK: Camera Utility API

/* Libere seulement les ressources internes de la camera. */
void M3D_end(Moteur3D* moteur);

/* Projection compacte monde -> ecran. */
M3D_PointProjection M3D_project_point(const Moteur3D* moteur, const Vect3* world_point);


//MARK: Camera Mode API

/* Modifie la distance a la cible en mode orbit. */
void M3D_zoom_orbit(Moteur3D* moteur, float delta);

/* Change le mode actif de la camera (FPS <-> ORBIT). */
void M3D_set_mode(Moteur3D* moteur, CameraMode mode);

//MARK: Input Binding API

/* Bindings clavier/souris par defaut. */
void M3D_bind_default_key_down(Moteur3D* moteur, M3D_InputState* input, int keycode, int is_repeat, int* should_quit);
void M3D_bind_default_key_up(M3D_InputState* input, int keycode);
void M3D_bind_default_mouse_motion(M3D_InputState* input, float dx, float dy);
void M3D_bind_default_mouse_wheel(Moteur3D* moteur, float wheel_y);

//MARK: Input Apply API

/* Applique l'etat d'input d'une frame avec delta time (secondes) et reset les deltas souris. */
void M3D_apply_input_state(Moteur3D* moteur, M3D_InputState* input, float delta_seconds);

//MARK: Advanced API Gate

/*
 * API avancee (bas niveau):
 * accessible uniquement si ADVANCED_API est defini avant cet include.
 */
#if defined(ADVANCED_API)
void M3D_move_camera_right(Moteur3D* moteur, float speed);
void M3D_move_camera_left(Moteur3D* moteur, float speed);
void M3D_move_camera_up(Moteur3D* moteur, float speed);
void M3D_move_camera_down(Moteur3D* moteur, float speed);
void M3D_move_camera_forward(Moteur3D* moteur, float speed);
void M3D_move_camera_backward(Moteur3D* moteur, float speed);

void M3D_rotate_camera_yaw(Moteur3D* moteur, float delta_deg);
void M3D_rotate_camera_pitch(Moteur3D* moteur, float delta_deg);
#endif

#endif // MOTEUR3D_H
