#include "pathtracer.h"

#include <iostream>

#include <Eigen/Dense>

#include <util/CS123Common.h>
#include <random>

using namespace Eigen;

PathTracer::PathTracer(int width, int height)
    : m_width(width), m_height(height), emmisionMultiplier( 5.0)
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
    using namespace std::chrono;
    auto start = high_resolution_clock::now();

    std::vector<Vector3f> intensityValues(m_width * m_height);
    setLightSources(scene);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();
    for(int y =0; y < m_height; ++y) {
        #pragma omp parallel for
        for(int x = 0 ; x < m_width; ++x) {
            int offset = x + (y * m_width);
            int samples = 5000;
            for (int rays=0; rays<samples; ++rays)
            {
                  intensityValues[offset] += tracePixel(x, y, scene, invViewMat) ;
            }


           intensityValues[offset] /= samples;

        }
    }

    toneMap(imageData, intensityValues);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::minutes>(stop - start);
    std::cout << "Time taken by function: "
            << duration.count() << " minutes" << std::endl;
}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
{


        Vector3f p(0, 0, 0);

       // view port space

       Vector2f myrand = (Vector2f::Random () + Vector2f::Ones())/2;
       float offsetX = 0.5 - myrand.x();
       float offsetY = 0.5 - myrand.y();
       Vector3f d(((2.f * (x+ offsetX) / m_width) - 1),
                       (1 - (2.f * (y+ offsetY) / m_height)), -1);

       d.normalize();

       //d is direction
       Ray r(p, d);
        r = r.transform(invViewMatrix);
       // }

        // world space
        return traceRay(r, scene);


}

Eigen::Vector3f PathTracer::getBRDF(const Vector3f& wi,  Vector3f& wo,
                                    const Vector3f& normal,const tinyobj::material_t& mat)
{

    const tinyobj::real_t *d = mat.diffuse;//Diffuse color as array of floats
    Vector3f diffuseV(d[0],d[1],d[2]);

    const tinyobj::real_t *s = mat.specular;//specular color as array of floats
    Vector3f specularV(s[0],s[1],s[2]);

    const tinyobj::real_t shininnes = mat.shininess;

   if(specularV.norm() > 0.0)
   {
       Vector3f reflectV(0.0,0.0,0.0);
       if(wi.dot(normal) < 0)
       {
         reflectV = wi - 2.0 * (wi.dot(normal)) * normal;
       }else
       {
         reflectV = ((2 * wi.dot(normal) * normal) - wi);
       }
       reflectV = reflectV.normalized();
       float n = ((shininnes + 2) / (2 * M_PI));
       float reflectShininnes = pow(reflectV.dot(wo), shininnes);
       return specularV * n * reflectShininnes;
   }
   else
   {
         return diffuseV/M_PI;
   }



}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene, int bounce )
{

    IntersectionInfo interesction;
    Ray ray(r);
    if(scene.getIntersection(ray, &interesction)) {

        //** Example code for accessing materials provided by a .mtl file **
        const Triangle *t = static_cast<const Triangle *>(interesction.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh

        const tinyobj::real_t *e = mat.emission;
        Vector3f pvEmission(e[0],e[1],e[2]);

        const tinyobj::real_t *d = mat.diffuse;//Diffuse color as array of floats

        bool isMirror = false;
        bool refrct = false;
        if (mat.illum == 5)
        {
             isMirror = true;
        }
        else if (mat.illum == 7)
        {
             refrct = true;
        }

        Vector3f diffuseV(d[0],d[1],d[2]);

        Vector3f px = interesction.hit;

        // I dont trust r.d anymore !!
        Vector3f currentDir(ray.d);

        const Vector3f hitNormal =  t->getNormal(px).normalized();

        Vector2f myrand = (Vector2f::Random () + Vector2f::Ones())/2.0;

        // hemi sphere sampling
        Vector3f newDir = uniformHemiSphere(hitNormal,myrand.x(),myrand.y());

        Vector3f brdf = getBRDF(newDir, currentDir,hitNormal,mat);

        Vector3f L  = directLight(scene,px,hitNormal).array() * brdf.array();

        Vector2f random = (Vector2f::Random () + Vector2f::Ones())/2.0;

        float pdf_rr =0.9;
        if(random.x() < pdf_rr)
        {
            if (isMirror)
            {
             Vector3f lr = doMirror(currentDir,hitNormal,px,scene);
              Vector3f incoming = lr / pdf_rr;
              L += incoming;
            }
            else if(refrct)
            {
               Vector3f lr = Refraction(currentDir, hitNormal,px, mat.ior,scene);
               lr /= pdf_rr;
                L += lr;
            }
            else
            {
               float costheta = newDir.dot(hitNormal);
               float pdf = 1.0 / (2.0 * M_PI);

               Vector3f lr = traceRay(Ray(px,newDir),scene,bounce+1)  ;

               Vector3f color = (lr.array()   * brdf.array());
               Vector3f incoming  = color    * costheta/ (pdf * pdf_rr) ;
               L +=  incoming  ;
            }

        }

        if(bounce == 0 )
         {
            L +=  pvEmission *emmisionMultiplier;
         }
        return L;

    } else {
        //std::cout << "NOT HIT     2" << std::endl;
        return Vector3f(0.0, 0.0, 0.0);
    }
}

Vector3f PathTracer::doMirror(const Vector3f& w, const Vector3f& n , const Vector3f& hitPos,const Scene& scene)
{
    Vector3f reflectVector = w - 2.0 * (w.dot(n)) * n;
    return traceRay(Ray(hitPos, reflectVector), scene, 0);
}

void PathTracer::createNormalPlane(const Vector3f & normal, Vector3f & v2,Vector3f & v3)
{
    /*if (std::fabs(normal.x()) > std::fabs(normal.y()))
    {
        v2 = Vector3f(normal.z(), 0, -normal.x()) / sqrtf(normal.x() * normal.x() + normal.z() * normal.z());
    }
    else{
        v2 = Vector3f(0, -normal.z(), normal.y()) / sqrtf(normal.y() * normal.y() + normal.z() * normal.z());

    }*/
   const Vector3f v1 = normal;
   if (std::fabs(v1.x()) > std::fabs(v1.y()))
    {
       float invLen = 1.f / sqrtf(v1.x() * v1.x() + v1.z() * v1.z());
       v2 = Vector3f(v1.z()* invLen, 0.0f, -v1.x() * invLen);
    }
    else{
        float invLen = 1.f / sqrtf(v1.y() * v1.y() + v1.z() * v1.z());
       v2 = Vector3f(0.0, -v1.z() * invLen, v1.y()*invLen); // sqrtf(normal.y() * normal.y() + normal.z() * normal.z());
    }
    v3 = (v1.cross(v2)).normalized();
}

Vector3f PathTracer::uniformHemiSphere(const Vector3f& normal, float x, float y)
{

   /*float phi = 2 * M_PI * x;
   float theta  = acosf(1-y);
   return Vector3f(sinf(theta)* cosf(phi), sinf(theta)*sinf(phi),cosf(theta)).normalized();*/


     // hemisphere uniform sampling from http://www.kevinbeason.com/smallpt/
    float r1 = 2.0f * M_PI *x;
    float r2 = y;
    float r2s = sqrt(r2);

    Vector3f u;
    if (fabs(normal.x()) > 0.1f)
    {
         u = Vector3f(0.0,1.0f,0.0).cross( normal);
    }
    else
    {
         u = Vector3f(1.0f, 0.0f, 0.0f).cross(normal);
    }

     u = u.normalized();
     Vector3f v = normal.cross( u);
     Vector3f newDir(u*cos(r1)*r2s + v*sin(r1)*r2s + normal*sqrt(1 - r2));
     return newDir.normalized();

}

Vector3f PathTracer::directLight(const Scene& scene,
                         const Vector3f& hitPos, const Vector3f& hitNormal )
{

      Vector3f vdirectLight(0.0,0.0,0.0) ;

      for(size_t i = 0 ; i < triangleLigthSources.size(); i++)
      {

          const Triangle*  t = triangleLigthSources[i];

          Vector3f v1(0.0,0.0,0.0);
          Vector3f v2(0.0,0.0,0.0);
          Vector3f v3(0.0,0.0,0.0);

          t->getVertices(v1,v2,v3);

          Vector3f lightColor(t->getMaterial().emission[0],t->getMaterial().emission[1],t->getMaterial().emission[2]);
          Vector3f lightDiffuse(t->getMaterial().diffuse[0],t->getMaterial().diffuse[1],t->getMaterial().diffuse[2]);


          lightColor *= emmisionMultiplier;

          Vector3f v12 = v2-v1;
          Vector3f v13 = v3-v1;

          float triangleArea = std::abs(v12.cross(v13).norm())/2;

          Vector3f accum(0.0,0.0,0.0);

          int samples = 5;
          for(int j = 0 ; j < samples; j++)
          {
               Vector2f myrand = (Vector2f::Random () + Vector2f::Ones())/2;
               float r1 = myrand.x();
               float r2 = myrand.y();

               Vector3f lightPosition = (1 - sqrt(r1)) * v1
                                         + (sqrt(r1) * (1 - r2)) * v2
                                         + (r2 * sqrt(r1)) * v3;



               Vector3f lightDir = (hitPos -lightPosition).normalized() ;
               Vector3f lightDirPrime = (lightPosition - hitPos).normalized();

               Ray r(hitPos, lightDirPrime);
               IntersectionInfo interesction;
               if(scene.getIntersection(r, &interesction)
                       && (interesction.hit - lightPosition).norm() < 0.001)
               {

                 Vector3f ligthHitPos =interesction.hit;
                 Vector3f normalV = t->getNormal(ligthHitPos).normalized();
                 float dis = (hitPos -lightPosition).norm();

                 float costheta = std::fmax(0.0,lightDirPrime.dot(hitNormal));
                 float costhetaPrime = std::fmax(0.0,lightDir.dot(normalV));


                 Vector3f vs = lightColor * costheta * costhetaPrime/(dis * dis)  ;
                 accum += vs;
               }

           }

          vdirectLight += (triangleArea / (float)samples) * accum;

      }



     return  vdirectLight ;


}

Vector3f PathTracer::Refraction(const Vector3f& w, const Vector3f& n,
                                const Vector3f& hitPos, float indexRfl, const Scene& scene)
{

    Vector3f refractVector = doRefract(w, n, indexRfl);
    float fresnelIndex = doFresnel(w, n, indexRfl);
    Vector2f myrand = (Vector2f::Random () + Vector2f::Ones())/2.0;
    if (myrand.x() > fresnelIndex)
    {
      return traceRay(Ray(hitPos, refractVector), scene, 0);

    }
    else
    {
       Vector3f reflectVector = w - 2.0 * (w.dot(n)) * n;
       return traceRay(Ray(hitPos, reflectVector), scene, 0);
    }
}

Vector3f PathTracer::doRefract(const Vector3f& w, const Vector3f& hitNormal, float indexRfl)
{
     Vector3f normal( hitNormal);
     float cosThetai = w.dot(hitNormal);

     float index =0.0;
     if (cosThetai < 0)
     {
          index = 1 / indexRfl;
          cosThetai *= -1;
     }
     else
     {
        index = indexRfl;
        normal *= -1;
     }
     float cosThetat = sqrtf(1.0 - index * index * (1.0 - cosThetai * cosThetai));
     Vector3f refractVector = index * w + (index * cosThetai - cosThetat) * normal;
     return refractVector.normalized();

}

float PathTracer::doFresnel(const Vector3f& w, const Vector3f& hitNormal, float indexRfl)
{
    float ni = 1;
    float nt = indexRfl;
    float cosThetai = w.dot(hitNormal);
    if (cosThetai < 0)
    {
         cosThetai = -cosThetai;
    }
    else
    {
         ni = indexRfl;
         nt = 1;
    }

    float vI = powf((ni - nt) / (ni + nt), 2);
    float VO = vI + (1.0 - vI) * powf(1.0 - cosThetai, 5);
    return VO;
}

void PathTracer::toneMap(QRgb *imageData, std::vector<Vector3f> &intensityValues) {
    for(int y = 0; y < m_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            Vector3f currentIntensity = intensityValues[offset];
            /*qRgb tonedColr(intensityValues[offset].x() / (1.0 + intensityValues[offset].x()) *255.0,
                           intensityValues[offset].y() / (1.0 + intensityValues[offset].y())*255.0,
                           intensityValues[offset].z() / (1.0 + intensityValues[offset].z())*255.0)*/

            imageData[offset] = qRgb(intensityValues[offset].x() / (1.0 + intensityValues[offset].x()) *255.0,
                                     intensityValues[offset].y() / (1.0 + intensityValues[offset].y())*255.0,
                                     intensityValues[offset].z() / (1.0 + intensityValues[offset].z())*255.0);
        }
    }

}
