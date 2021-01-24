//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
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
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

inline bool equal(const Vector3f &v1, const Vector3f &v2)
{
    return std::abs(v1.x - v2.x) < 10 * EPSILON &&
           std::abs(v1.y - v2.y) < 10 * EPSILON &&
           std::abs(v1.z - v2.z) < 10 * EPSILON;
}
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    //direct light
    Intersection ray_inter = intersect(ray);
    if (!ray_inter.happened)
    {
        return Vector3f();
    }

    Vector3f L_direct;
    Vector3f wo = normalize(-ray.direction);
    Intersection light_inter;
    float pdf_light;
    sampleLight(light_inter, pdf_light);
    Vector3f ws = normalize(light_inter.coords - ray_inter.coords);
    light_inter.distance = (light_inter.coords - ray_inter.coords).norm();

    Intersection test_inter = intersect(Ray(ray_inter.coords, ws));
    if (fabs(light_inter.distance - test_inter.distance) < 10 * EPSILON)
    {
        L_direct = light_inter.emit * ray_inter.m->eval(ws, wo, ray_inter.normal) * dotProduct(ws, ray_inter.normal) * dotProduct(-ws, light_inter.normal) / (dotProduct(ray_inter.coords - light_inter.coords, ray_inter.coords - light_inter.coords) * (pdf_light + EPSILON));
    }

    Vector3f L_indir;
    if (false) //get_random_float() < RussianRoulette)
    {
        // std::cout << "Win RR!\n";
        Vector3f wi = normalize(ray_inter.m->sample(wo, ray_inter.normal));
        Intersection object_inter = intersect(Ray(ray_inter.coords, wi));
        if (object_inter.happened && !object_inter.obj->hasEmit())
        {
            // std::cout << "Hey!\n";
            L_indir = castRay(Ray(ray_inter.coords, wi), 0) * ray_inter.m->eval(wo, wi, ray_inter.normal) * dotProduct(wi, ray_inter.normal) / (ray_inter.m->pdf(wo, wi, ray_inter.normal) * RussianRoulette);
        }
    }

    return L_direct + L_indir;
}