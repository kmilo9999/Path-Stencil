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

void PathTracer::setLightSources(const Scene &scene)
{
   const std::vector<Object *> sceneObjects = scene.sceneObjects();

   for(size_t i =  0 ; i < sceneObjects.size(); i++)
   {
       const Mesh * m = static_cast<const Mesh *> (sceneObjects[i]);
       const Triangle* pt = m->triangles();
       size_t numT = m->getNumTriangles();
       std::cout << numT << std::endl;
       for(size_t i = 0 ; i < numT ; i++)
       {
          const Triangle& t = pt[i];
          const tinyobj::material_t& mat = t.getMaterial();
          const tinyobj::real_t *e = mat.emission;
          Vector3f pvEmission(e[0],e[1],e[2]);
          if(pvEmission != Vector3f::Zero())
          {
                triangleLigthSources.push_back(&t);
          }


       }

   }
   std::cout << "DONE" << std::endl;
   std::cout << "num emmissive triangles: " << triangleLigthSources.size() << std::endl;
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    std::vector<Vector3f> intensityValues(m_width * m_height);
    setLightSources(scene);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();
    for(int y = 0; y < m_height; ++y) {
        //#pragma omp parallel for
        //std::cout << y << std::endl;
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
           // intensityValues[offset] = tracePixel(x, y, scene, invViewMat);

            intensityValues[offset] = Vector3f(0.0,0.0,0.0);
            for (int rays=0; rays<100; ++rays)
            {
                //intensityValues[offset] += tracePixel(115, 49, scene, invViewMat);
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
   // for(int pxs=0;pxs<50;pxs++)
   // {
        // view port space

        Vector2f myrand = (Vector2f::Random () + Vector2f::Ones())/2;
        //Vector3f randomRay(2.0 * distribution(generator) - 1,1 - (2.f * distribution(generator) / m_height),0);
        Vector3f d(((2.f * (x+ myrand.x()) / m_width) - 1),
                   (1 - (2.f * (y+ myrand.y()) / m_height)), -1);

        d.normalize();

        //d is direction
        Ray r(p, d);
        r = r.transform(invViewMatrix);
   // }



    // world space



    return traceRay(r, scene);

}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene, int bounce )
{



    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0,1);
    float random = distribution(generator);

    if(bounce >= 5)
    {
        if( random <= 0.1)
        {
            return  Vector3f(0, 0, 0);
        }
    }

    IntersectionInfo interesction;
    Ray ray(r);
    if(scene.getIntersection(ray, &interesction)) {

        //** Example code for accessing materials provided by a .mtl file **
        const Triangle *t = static_cast<const Triangle *>(interesction.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
        const std::vector<CS123SceneLightData> sceneLights = const_cast<Scene&>(scene).getLights();
        const tinyobj::real_t *e = mat.emission;
        Vector3f pvEmission(e[0],e[1],e[2]);

        const tinyobj::real_t *d = mat.diffuse;//Diffuse color as array of floats
        Vector3f diffuseV(d[0],d[1],d[2]);

        Vector3f px = interesction.hit;
        const Vector3f hitNormal =  t->getNormal(interesction);

        Vector3f L = Vector3f::Zero();
        Vector3f dL  = directLight(scene,px,hitNormal,diffuseV);
         //Vector3f L  = diffuseV + pvEmission;



        Vector2f myrand = (Vector2f::Random () + Vector2f::Ones())/2;
        Vector3f sample = uniformHemiSphere(myrand.x(), myrand.y());
        Vector3f v1(0.0,0.0,0.0);
        Vector3f v2(0.0,0.0,0.0);
        createNormalPlane(hitNormal,v1,v2);
        Vector3f wi(0.0,0.0,0.0);
        wi[0] = sample.x() * v2.x() + sample.y() * hitNormal.x() + sample.z() * v1.x();
        wi[1] = sample.x() * v2.y() + sample.y() * hitNormal.y() + sample.z() * v1.y();
        wi[2]= sample.x() * v2.z() + sample.y() * hitNormal.z() + sample.z() * v1.z();
        ray.o = px;
        ray.d = wi;
        float costheta = std::fmax(0.0,ray.d.dot(hitNormal));
        float pdf = 1 / (2 * M_PI);



        //Vector3f lr = traceRay(ray,scene,bounce+1)  ;
        //Vector3f  brdf = diffuseV/M_PI;

        //Vector3f incoming  =   (lr.array()   * brdf.array()) * costheta / (pdf * 0.1) ;

        //L =  incoming  ;
        //L = dL + incoming;
        L = dL;
        if(bounce == 0 )
        {
           L +=  (pvEmission);
        }
        return L;

    } else {
        //std::cout << "NOT HIT     2" << std::endl;
        return Vector3f(0, 0, 0);
    }
}


void PathTracer::createNormalPlane(const Vector3f & normal, Vector3f & v2,Vector3f & v3)
{
    if (std::fabs(normal.x()) > std::fabs(normal.y()))
    {
        v2 = Vector3f(normal.z(), 0, -normal.x()) / sqrtf(normal.x() * normal.x() + normal.z() * normal.z());
    }
    else{
        v2 = Vector3f(0, -normal.z(), normal.y()) / sqrtf(normal.y() * normal.y() + normal.z() * normal.z());

    }
    v3 = normal.cross(v2);
}

Vector3f PathTracer::uniformHemiSphere(float x, float y)
{
   float r = sqrtf(1.0-x*x);
   float phi = 2*M_PI*y;
   return Vector3f(cosf(phi) * r,x,sinf(phi) *r);
}

Vector3f PathTracer::directLight(const Scene& scene,
                         const Vector3f& hitPos, const Vector3f& hitNormal, Vector3f hitColor )
{

    Vector3f directLight(0.0,0.0,0.0) ;
    //for(size_t i = 0 ; i < sceneLights.size();i++)
    for(size_t i = 0 ; i < triangleLigthSources.size();i++)
    {
      const Triangle*  t = triangleLigthSources[i];

      Vector3f v1(0.0,0.0,0.0);
      Vector3f v2(0.0,0.0,0.0);
      Vector3f v3(0.0,0.0,0.0);

      Vector3f n1(0.0,0.0,0.0);
      Vector3f n2(0.0,0.0,0.0);
      Vector3f n3(0.0,0.0,0.0);

      t->getVertices(v1,v2,v3);
      t->getVertices(n1,n2,n3);
      std::vector<Vector3f> vVertices;
      std::vector<Vector3f> vNormals;
      vVertices.push_back(v1);
      vVertices.push_back(v2);
      vVertices.push_back(v3);
      vNormals.push_back(n1);
      vNormals.push_back(n2);
      vNormals.push_back(n3);


      Vector3f lightColor(t->getMaterial().emission[0],t->getMaterial().emission[1],t->getMaterial().emission[2]);
      Vector3f lightDiffuse(t->getMaterial().diffuse[0],t->getMaterial().diffuse[1],t->getMaterial().diffuse[2]);
      Vector3f lightAmbient(t->getMaterial().ambient[0],t->getMaterial().ambient[1],t->getMaterial().ambient[2]);

      Vector3f v12 = v2-v1;
      Vector3f v13 = v3-v1;
      float triangleArea = std::abs(v12.cross(v13).norm())/2;

      Vector3f accum(0.0,0.0,0.0);
      for(size_t j = 0 ; j < 1;j++)
      {

          Vector3f lightPosition = vVertices[j];

          //Vector3f normalV = t->getNormal(lightPosition);
          Vector3f normalV =vNormals[j];


          Vector3f lightDir = (hitPos -lightPosition).normalized() ;

          int visibility = 0;

          Ray r(hitPos, lightDir);
          IntersectionInfo interesction;
          if(scene.getIntersection(r, &interesction))
          {
            visibility = 1 ;
          }


          Vector3f lightDirPrime = (lightPosition - hitPos).normalized() ;

          float costheta = std::fmax(0.0,hitNormal.dot(lightDirPrime));
          float costhetaPrime = std::fmax(0.0,normalV.dot(-lightDir));

          float lenght = (hitPos -lightPosition).norm();
          float lenght2 = lenght * lenght;


          Vector3f light = ( lightColor ) * (1/M_PI) * visibility
                  * costheta*costhetaPrime* (1 / lenght2) * triangleArea  ;
         // light = light /  1 + light;


          accum = (hitColor.array() * light.array()  ) ;
      }
         //Vector3f s =  accum.array() ;
         directLight += accum;
         //directLight *= 1/3  ;
    }

    return directLight/2  ;

}

void PathTracer::toneMap(QRgb *imageData, std::vector<Vector3f> &intensityValues) {
    for(int y = 0; y < m_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            Vector3f currentIntensity = intensityValues[offset];

            /*imageData[offset] = currentIntensity.norm() > 0 ? qRgb(255.0, 255.0, 255.0)
                                                            : qRgb(255.0, 40.0, 40.0 );*/
            /*imageData[offset] = qRgb(currentIntensity[0] *255.0,
                                     currentIntensity[1]*255.0,
                                     currentIntensity[2]*255.0);*/
            Vector3f tonedColor = currentIntensity.array() / (Vector3f(1,1,1) + currentIntensity).array();
            imageData[offset] = qRgb(tonedColor[0] *255.0,
                                                 tonedColor[1]*255.0,
                                                 tonedColor[2]*255.0);
        }
    }

}
