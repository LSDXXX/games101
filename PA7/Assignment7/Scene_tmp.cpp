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
    auto hit_point = hit.coords;
    auto N = normalize(hit.normal);
    if(hit.happened&&isValidF(hit_point.x)&&isValidF(hit_point.y)&&isValidF(hit_point.z)) {
        if(hit.obj->hasEmit()) {
            if(depth == 0) hit_color= hit.emit;
            return hit_color;
        }
        
        Intersection sample_light;
        float pdf_light = 0.f;
        sampleLight(sample_light, pdf_light);
        auto pos2light = sample_light.coords - hit_point;
        auto light_distance = pos2light.norm();
        auto wi = normalize(pos2light);
        auto wo = -ray.direction;
        Vector3f L_dir(0.f);
        auto shadow = intersect(Ray(hit_point, wi));
        float dist_shadow = (shadow.coords-sample_light.coords).norm();
        if(dist_shadow < 0.01) {
            L_dir = sample_light.emit*hit.m->eval(wi, wo, N)*dotProduct(N, wi)*
                -dotProduct(sample_light.normal, wi)/(light_distance*light_distance)/pdf_light;
                /*
            std::cout << sample_light.emit << ", " 
            << hit.m->eval(wi, wo, N) << ", " << dotProduct(N, wi) 
            << ", " << -1*dotProduct(sample_light.normal, wi) 
            << std::endl;
            */
        }

        Vector3f L_indir(0.f);
        if(get_random_float() < RussianRoulette) {
            wi = hit.m->sample(wo, N);
            auto pdf = hit.m->pdf(wi, wo, N);
            auto shade = castRay(Ray(hit_point, wi), depth+1);
            L_indir = hit.m->eval(wi, wo, N)*shade*dotProduct(wi,N)/pdf/RussianRoulette;
        }

        hit_color= L_dir + L_indir;
    }
    return hit_color;
}