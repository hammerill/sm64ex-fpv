// explosion.c.inc

void bhv_explosion_init(void) {
    create_sound_spawner(SOUND_GENERAL2_BOBOMB_EXPLOSION);
    set_environmental_camera_shake(SHAKE_ENV_EXPLOSION);

    o->oOpacity = 255;
}

// free object slots, counted up to `max`
static s32 count_free_object_slots(s32 max) {
    s32 n = 0;
    struct ObjectNode *node = gFreeObjectList.next;
    while (node != NULL && n < max) {
        n++;
        node = node->next;
    }
    return n;
}

void bhv_explosion_loop(void) {
    s32 i;
    s32 bubbles;

    if (o->oTimer == 9) {
        if (find_water_level(o->oPosX, o->oPosZ) > o->oPosY) {
            // 40 bubbles each, so a few explosions underwater at once could fill the object
            // pool and hang allocate_object; keep some slots free for everything else
            bubbles = count_free_object_slots(40 + 30) - 30;
            for (i = 0; i < bubbles; i++)
                spawn_object(o, MODEL_WHITE_PARTICLE_SMALL, bhvBobombExplosionBubble);
        } else
            spawn_object(o, MODEL_SMOKE, bhvBobombBullyDeathSmoke);

        o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
    }

    o->oOpacity -= 14;

    cur_obj_scale((f32) o->oTimer / 9.0f + 1.0);
}
