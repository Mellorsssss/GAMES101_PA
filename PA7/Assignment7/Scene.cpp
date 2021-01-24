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
    return std::abs(v1.x - v2.x) < 2 &&
           std::abs(v1.y - v2.y) < 2 &&
           std::abs(v1.z - v2.z) < 2;
}
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // direct light
    Intersection ray_inter = intersect(ray);
    if (!ray_inter.happened)
    {
        // std::cout << "Hit nothing!\n";
        return Vector3f();
    }

    Vector3f L_direct;
    Vector3f wo = -ray.direction;
    Intersection light_inter;
    float pdf_light;
    sampleLight(light_inter, pdf_light);
    // std::cout << "object pos :" << ray_inter.coords << std::endl;
    // std::cout << "Light pos :" << light_inter.coords << std::endl;
    Vector3f ws = light_inter.coords - ray_inter.coords;

    Intersection test_inter = intersect(Ray(ray_inter.coords, ws));

    std::cout << test_inter.coords << "  " << light_inter.coords << std::endl;
    if (equal(test_inter.coords, light_inter.coords))
    {
        L_direct = test_inter.emit * ray_inter.m->eval(ws, wo, ray_inter.normal) * dotProduct(ws, ray_inter.normal) * dotProduct(-ws, light_inter.normal) / (dotProduct(ray_inter.coords - light_inter.coords, ray_inter.coords - light_inter.coords) * pdf_light);
        std::cout << L_direct << std::endl;
    }
    else
    {
        ; // std::cout << "Blocked!\n";
    }

    return L_direct;

    // intersect(ray())
    // indirect light
}