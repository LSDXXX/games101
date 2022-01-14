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
    Vector3f hit_color(0.f);
    auto hit = this->intersect(ray);
    if(hit.happened) {
        if(hit.obj->hasEmit()) {
            return hit.emit;
        }
        Vector3f L_light(0.0f), L_object(0.0f);
        // calculate light
        float pdf_light;
        Intersection light_sample;
        sampleLight(light_sample, pdf_light);
        auto pos_to_light_dir = light_sample.coords - hit.coords;
        auto pos_to_light_distance = std::sqrt(dotProduct(pos_to_light_dir, pos_to_light_dir));
        pos_to_light_dir = normalize(pos_to_light_dir);
        Ray pos_to_light_ray(hit.coords, pos_to_light_dir);
        auto is_shadow = this->intersect(pos_to_light_ray);
        // std::cout << is_shadow.happened << ", " << is_shadow.distance << ", " << pos_to_light_distance << std::endl;
        if(!is_shadow.happened || is_shadow.distance > pos_to_light_distance) {
            auto cos_theta = dotProduct(hit.normal, pos_to_light_dir);
            auto fr = hit.m->eval(-1*pos_to_light_dir,-1*ray.direction, hit.normal);
            auto cos_theta_ = -dotProduct(light_sample.normal, pos_to_light_dir);
            L_light = light_sample.emit*cos_theta*cos_theta_*fr/pdf_light/(pos_to_light_distance*pos_to_light_distance);
        }

        // calculate diffuse object
        Intersection diffuse_sample;
        auto dir = hit.m->sample(ray.direction, hit.normal);
        float pdf_diffuse = hit.m->pdf(ray.direction, dir, hit.normal);
        if(!hit.obj->hasEmit() && get_random_float() < RussianRoulette) {
            auto fr = hit.m->eval(-1*dir, -1*ray.direction, hit.normal);
            auto cos_theta = dotProduct(hit.normal, dir);
            L_object = fr*cos_theta*this->castRay(Ray(hit.coords, dir), depth+1)/pdf_diffuse/RussianRoulette;
        }
        hit_color = L_light + L_object;
    }
    return hit_color;
}