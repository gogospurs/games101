//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    const float EPSILON = 0.0001;

    Intersection inter_p = Scene::intersect(ray);
    if (inter_p.happened == false)    return Vector3f(0.0);
    if (inter_p.m->hasEmission()) return inter_p.m->getEmission();
    
    Vector3f dir_light(0.0), indir_light(0.0);

    Intersection inter_light;
    float pdf_light{0};
    sampleLight(inter_light, pdf_light);
    
    Vector3f hitPoint_p = inter_p.coords, hitPoint_light = inter_light.coords;
    Vector3f wi = ray.direction;
    Vector3f light_dir = (hitPoint_light - hitPoint_p).normalized();
    float p_light_dis = (hitPoint_light - hitPoint_p).norm();
    Vector3f p_normal = inter_p.normal.normalized(), light_normal = inter_light.normal.normalized();

    Ray ray_to_light(hitPoint_p, light_dir);
    Intersection inter_to_light = Scene::intersect(ray_to_light);
    if (inter_to_light.happened && inter_to_light.distance > -EPSILON+p_light_dis) {
        Vector3f Li = inter_light.emit;
        Vector3f f_r = inter_p.m->eval(-wi, light_dir, p_normal);
        float cos_theta = dotProduct(light_dir, p_normal);
        float cos_theta_prime = dotProduct(-light_dir, light_normal);
        dir_light = Li * f_r * cos_theta * cos_theta_prime / (p_light_dis * p_light_dis * pdf_light);
    }

    float random_number = get_random_float();
    if (random_number <= RussianRoulette) {
        Vector3f wo = inter_p.m->sample(wi, p_normal).normalized();
        Ray ray_trace(hitPoint_p, wo);
        Intersection inter_trace = Scene::intersect(ray_trace);
        if (inter_trace.happened && !(inter_trace.m->hasEmission())) { 
            Vector3f indir_light_intensity = castRay(ray_trace, depth+1);
            Vector3f f_r = inter_p.m->eval(-wi, wo, p_normal);
            float cos_theta = dotProduct(wo, p_normal);
            float pdf_hemi = inter_p.m->pdf(wi, wo, p_normal);
            indir_light = indir_light_intensity * f_r * cos_theta / pdf_hemi / RussianRoulette;
        }
    }
    return indir_light + dir_light;
}