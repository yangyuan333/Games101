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
    
    Vector3f color{ 0.0,0.0,0.0 };

    Intersection inter = intersect(ray);
    if (inter.emit.norm() > 0.0) {
        // hit the light
        return Vector3f{ 1.0,1.0,1.0 };
    }
    else if (inter.happened) {
        // hit the object
        // direct sample the light
        Vector3f dir_color{ 0.0,0.0,0.0 };
        Intersection inter_light; float light_pdf;
        sampleLight(inter_light, light_pdf);

        Vector3f wi_d = inter_light.coords - inter.coords;
        Vector3f wi = normalize(wi_d);
        Intersection inter_object = intersect(Ray{ inter.coords ,wi });
        Vector3f wo = -ray.direction;
        Vector3f N = inter.normal;
        if ((inter_object.coords - inter_light.coords).norm() < 0.0001) {
            // hit the light
            color += inter_light.emit * inter.m->eval(wi, wo, N) *
                dotProduct(inter_light.normal, -wi) * dotProduct(N, wi) /
                dotProduct(wi_d, wi_d) / light_pdf;
        }
        // indirect sample the object
        float prob = get_random_float();
        if (prob < RussianRoulette) {
            wi = inter.m->sample(wo, N);

            color += castRay(Ray{ inter.coords + N * EPSILON,wi }, depth + 1) * inter.m->eval(wi, wo, N) *
                dotProduct(wi, N) / inter.m->pdf(wi, wo, N) / RussianRoulette;
        }
    }
    return color;

}