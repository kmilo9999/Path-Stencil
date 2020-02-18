#include "pathtracer.h"

#include <iostream>

#include <Eigen/Dense>

#include <util/CS123Common.h>
#include <random>

using namespace Eigen;

PathTracer::PathTracer(int width, int height)
    : m_width(width), m_height(height)
{
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    std::vector<Vector3f> intensityValues(m_width * m_height);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();
    for(int y = 0; y < m_height; ++y) {
        //#pragma omp parallel for
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
           // intensityValues[offset] = tracePixel(x, y, scene, invViewMat);

            intensityValues[offset] = Vector3f(0.0,0.0,0.0);
            for (int rays=0; rays<50; ++rays)
            {
                intensityValues[offset] += tracePixel(x, y, scene, invViewMat);
            }
            intensityValues[offset] /= 50;

        }
    }

    toneMap(imageData, intensityValues);
}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
{
    Vector3f p(0, 0, 0);
    // view port space
    Vector3f d((2.f * x / m_width) - 1, 1 - (2.f * y / m_height), -1);
    d.normalize();

    //d is direction
    Ray r(p, d);
    // world space
    r = r.transform(invViewMatrix);

    Vector3f color(0,0,0);
    return traceRay(r, scene,color);

}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene,Vector3f color, int bounce )
{

    /*if(bounce > maxbounce)
    {
        maxbounce = bounce;
        std::cout<< "max bounce: " << maxbounce << std::endl;
    }*/
    std::mt19937 mersenneTwister;
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0,1);
    //float random = distribution(generator);
   // std::cout << "traceRay" << std::endl;
    if(bounce >= 3)
    {
        //if( random <= 0.1)
        //{
             //std::cout << "RETURNNNNNNNNNNNNNNNNNNN" << std::endl;
            return  Vector3f(0, 0, 0);
        //}
    }

    IntersectionInfo interesction;
    Ray ray(r);
    if(scene.getIntersection(ray, &interesction)) {


        //** Example code for accessing materials provided by a .mtl file **
        const Triangle *t = static_cast<const Triangle *>(interesction.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
        const tinyobj::real_t *e = mat.emission;
        Vector3f pvEmission(e[0],e[1],e[2]);

        const tinyobj::real_t *d = mat.diffuse;//Diffuse color as array of floats
        Vector3f diffuseV(d[0],d[1],d[2]);

        /*if(pvEmission != Vector3f::Zero())
        {
            std::cout << "EMISEVE" << std::endl;
        }*/
        Vector3f L = diffuseV + pvEmission;
        Vector3f px = interesction.hit;
        const Vector3f hitNormal =  t->getNormal(interesction);

        //CS123SceneColor directLight  = lEmmited(sceneLights,hitPoint,hitNormal);

        float r1 = distribution(generator);
        float r2 = distribution(generator);
        Vector3f newDir = uniformHemiSphere(r1, r2);
        Vector3f v1(0.0,0.0,0.0);
        Vector3f v2(0.0,0.0,0.0);
        createNormalPlane(hitNormal,v1,v2);
        Vector3f wi;
        wi[0] = Vector3f(v1.x(),  hitNormal.x() ,v2.x() ).dot(newDir);
        wi[1] = Vector3f(v1.y(), hitNormal.y() ,v2.y()).dot(newDir);
        wi[2]= Vector3f(v1.z(), hitNormal.z() ,v2.z() ).dot(newDir);
        ray.o = px;
        ray.d = wi;
        float costheta = std::fmaxf(0.0,ray.d.dot(hitNormal));
        float pdf = 1 / (2 * M_PI);
        float factor = costheta / (pdf);// * 0.1);
        Vector3f lr = traceRay(ray,scene,color,bounce+1) ;
        Vector3f tmp =  lr.array() * (diffuseV.array() / M_PI) *costheta ;
        L +=  tmp/pdf ;
        return L;
       //  return Vector3f(d[0] , d[1], d[2]);
    } else {
        //std::cout << "NOT HIT     2" << std::endl;
        return Vector3f(0, 0, 0);
    }
}


void PathTracer::createNormalPlane(const Vector3f & v1, Vector3f & v2,Vector3f & v3)
{
    if (std::abs(v1.x()) > std::abs(v1.y())) {
           // project to the y = 0 plane and construct a normalized orthogonal vector in this plane
           float invLen = 1.f / sqrtf(v1.x() * v1.x() + v1.z() * v1.z());
           v2 = Vector3f(-v1.z() * invLen, 0.0f, v1.x() * invLen);
       } else {
           // project to the x = 0 plane and construct a normalized orthogonal vector in this plane
           float invLen = 1.0f / sqrtf(v1.y() * v1.y() + v1.z() * v1.z());
           v2 = Vector3f(0.0f, v1.z() * invLen, -v1.y() * invLen);
       }
       v3 = v1.cross(v2);
}

Vector3f PathTracer::uniformHemiSphere(float x, float y)
{
   float r = sqrtf(1.0-x*x);
   float phi = 2*M_PI*y;
   return Vector3f(cosf(phi) * r,sinf(phi) *r,x);
}

CS123SceneColor PathTracer::lEmmited(const std::vector<CS123SceneLightData>& sceneLights,
                         const Vector3f& hitPos, const Vector3f& hitNormal )
{

    CS123SceneColor directLight ;
    for(size_t i = 0 ; i < sceneLights.size();i++)
    {

      Vector3f lightPosition(sceneLights[i].pos[0],sceneLights[i].pos[1],sceneLights[i].pos[2]);
      Vector3f lightDir = hitPos -lightPosition ;
      Vector4f lightColor(sceneLights[i].color.x(),sceneLights[i].color.y(),sceneLights[i].color.z(),1);
      float r2 = lightDir.norm();
      float distance = sqrtf(r2);
      lightDir /= distance;
      //std::cout << "HIT???????????" << std::endl;
      //lightIntensity =sceneLights[i].color * intensity / (4 * M_PI * r2);
      //lightIntensity = sceneLights[i].color * 100.0 / (4 * M_PI * r2);
     // std::cout << "HIT&&&&&&&&&&&" << std::endl;
      directLight = (lightColor * 10.0 / (4 * M_PI * r2) ) * std::max(0.f, hitNormal.dot(-lightDir));

    }

    return directLight;

}

void PathTracer::toneMap(QRgb *imageData, std::vector<Vector3f> &intensityValues) {
    for(int y = 0; y < m_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            Vector3f currentIntensity =intensityValues[offset];
            /*imageData[offset] = currentIntensity.norm() > 0 ? qRgb(255.0, 255.0, 255.0)
                                                            : qRgb(255.0, 40.0, 40.0 );*/
            imageData[offset] = qRgb(currentIntensity[0] *255.0,
                                     currentIntensity[1]*255.0,
                                     currentIntensity[2]*255.0);
        }
    }

}
