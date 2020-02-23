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
    for(int y =0; y < m_height; ++y) {
        //#pragma omp parallel for
        //std::cout << y << std::endl;
        for(int x = 0 ; x < m_width; ++x) {
            int offset = x + (y * m_width);
            int samples = 100;
            for (int rays=0; rays<samples; ++rays)
            {
                // intensityValues[offset] = tracePixel(x, y, scene, invViewMat);

                 //intensityValues[offset] = Vector3f(0.0,0.0,0.0);

                  //   intensityValues[offset] += tracePixel(188, 203, scene, invViewMat)/samples;
                     //  intensityValues[offset] += tracePixel(75, 143, scene, invViewMat)/samples;
                  intensityValues[offset] += tracePixel(x, y, scene, invViewMat) ;
            }


           intensityValues[offset] /= samples;

        }
    }

    toneMap(imageData, intensityValues);
}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
{


        Vector3f p(0, 0, 0);

       // view port space

       Vector2f myrand = (Vector2f::Random () + Vector2f::Ones())/2;
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

Eigen::Vector3f PathTracer::getBRDF(const Vector3f& rayDir, const Vector3f& normal,const tinyobj::material_t& mat)
{

    const tinyobj::real_t *d = mat.diffuse;//Diffuse color as array of floats
    Vector3f diffuseV(d[0],d[1],d[2]);

    const tinyobj::real_t *s = mat.specular;//specular color as array of floats
    Vector3f specularV(s[0],s[1],s[2]);

    const tinyobj::real_t shininnes = mat.shininess;

   if(specularV !=  Vector3f::Zero())
   {
       Vector3f reflectV = rayDir - 2.0 * (rayDir.dot(normal)) * normal;
       float reflectShininnes = pow(reflectV.dot(rayDir), shininnes);
       return specularV * ((shininnes + 2) / (2 * M_PI)) * reflectShininnes;
   }
   else
   {
         return diffuseV/M_PI;
   }



}

Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene, int bounce )
{

  //  std::cout << "bounce :" << bounce << std::endl;



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

        Vector3f px = interesction.hit;


        const Vector3f hitNormal =  t->getNormal(px);

        Vector2f myrand = (Vector2f::Random () + Vector2f::Ones())/2.0;

        // hemi sphere sampling
        Vector3f newDir = uniformHemiSphere(hitNormal,myrand.x(),myrand.y());

        Vector3f brdf = getBRDF(newDir,hitNormal,mat);
        //Vector3f brdf = diffuseV/M_PI;

        Vector3f L  = directLight(scene,px,hitNormal,diffuseV).array() * brdf.array();

        Vector2f random = (Vector2f::Random () + Vector2f::Ones())/2.0;

        float pdf_rr =0.5;
        if(random.x() < pdf_rr)
        {

            float costheta = newDir.dot(hitNormal);
            float pdf = 1.0 / (2.0 * M_PI);

            Vector3f lr = traceRay(Ray(px,newDir),scene,bounce+1)  ;

            Vector3f color = (lr.array()   * brdf.array());
            Vector3f incoming  = color    * costheta/ (pdf * pdf_rr)  ;
            L +=  incoming  ;
        }

        if(bounce == 0 )
         {
            L +=  pvEmission *5;
         }
        return L;

    } else {
        //std::cout << "NOT HIT     2" << std::endl;
        return Vector3f(0.0, 0.0, 0.0);
    }
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
                         const Vector3f& hitPos, const Vector3f& hitNormal, Vector3f hitColor )
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


          lightColor *=5;

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
               /*if(r1 +r2 >= 1)
               {
                 r1 = 1- r1;
                 r2 = 1- r2;
               }*/

               // Vector3f lightPosition = v1 + r1*(v12) + r2*(v13);
               Vector3f lightPosition = (1 - sqrt(r1)) * v1
                                         + (sqrt(r1) * (1 - r2)) * v2
                                         + (r2 * sqrt(r1)) * v3;

               Vector3f normalV = t->getNormal(lightPosition).normalized();

               Vector3f lightDir = (hitPos -lightPosition) ;
               Vector3f lightDirPrime = (lightPosition - hitPos);

               Ray r(hitPos, lightDirPrime);
               IntersectionInfo interesction;
               if(scene.getIntersection(r, &interesction)
                       && (interesction.hit - lightPosition).norm() < 0.001)
               {

                 float dis = (hitPos -lightPosition).norm();

                 float costheta = std::fmax(0.0,lightDirPrime.dot(hitNormal)/lightDirPrime.norm());
                 float costhetaPrime = std::fmax(0.0,lightDir.dot(normalV)/lightDirPrime.norm());


                 Vector3f vs = lightColor * costheta * costhetaPrime/(dis * dis)  ;
                 accum += vs;
               }

           }

          vdirectLight += (triangleArea / (float)samples) * accum;

      }



     return  vdirectLight ;


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
            //Vector3f tonedColor = currentIntensity.array() / (Vector3f(1,1,1) + currentIntensity).array();
            float r = intensityValues[offset].x() / (1.0 + intensityValues[offset].x());
            float g = intensityValues[offset].y() / (1.0 + intensityValues[offset].y());
            float b = intensityValues[offset].z() / (1.0 + intensityValues[offset].z());

            imageData[offset] = qRgb(r *255.0,
                                                 g*255.0,
                                                 b*255.0);
        }
    }

}
