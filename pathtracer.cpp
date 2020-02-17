#include "pathtracer.h"

#include <iostream>

#include <Eigen/Dense>

#include <util/CS123Common.h>

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

    //std::cout << "HIT     2" << std::endl;
    return traceRay(r, scene);

}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene)
{
    IntersectionInfo interesction;
    Ray ray(r);
    if(scene.getIntersection(ray, &interesction)) {

      //  std::cout << "HIT---------------" << std::endl;
        //** Example code for accessing materials provided by a .mtl file **
        const Triangle *t = static_cast<const Triangle *>(interesction.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
        const std::vector<CS123SceneLightData> sceneLights = const_cast<Scene&>(scene).getLights();
        CS123SceneColor lightIntensity;
        CS123SceneColor directLight ;
        const Vector3f hitNormal =  t->getNormal(interesction);
       // std::cout << "HIT***********" << std::endl;
        for(size_t i = 0 ; i < sceneLights.size();i++)
        {
          Vector3f hitPoint = interesction.hit;
          Vector3f lightPosition(sceneLights[i].pos[0],sceneLights[i].pos[1],sceneLights[i].pos[2]);
          Vector3f lightDir = hitPoint -lightPosition ;
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

      //  std::cout << "HIT%%%%%%%%%%%" << std::endl;
        const tinyobj::real_t *d = mat.diffuse;//Diffuse color as array of floats
       // std::cout << d[0] << " " << d[1] << " " << d[2] << std::endl;
//        const std::string diffuseTex = mat.diffuse_texname;//Diffuse texture name
       // std::cout << sizeof(d) / sizeof(float) << std::endl;
        //CS123SceneColor color(d[0], d[1], d[2],1 );
        //return Vector3f(d[0] ,d[1], d[2]);
    //     std::cout << "HIT++++++++++++++" << std::endl;
        return Vector3f(directLight.x() * d[0] , directLight.y() *d[1], directLight.z() *d[2]);
    } else {
       //   std::cout << "NO HIT" << std::endl;
        return Vector3f(0, 0, 0);
    }
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
