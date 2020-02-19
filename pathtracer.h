#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>

#include "scene/scene.h"


class PathTracer
{
public:
    PathTracer(int width, int height);

    void traceScene(QRgb *imageData, const Scene &scene);

private:
    int m_width, m_height;

    void toneMap(QRgb *imageData, std::vector<Eigen::Vector3f> &intensityValues);
    void lightSources(const Scene &scene);

    Eigen::Vector3f tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix);
    Eigen::Vector3f traceRay(const Ray& r, const Scene &scene,Eigen::Vector3f color, int bounce = 0);

    CS123SceneColor lEmmited(const std::vector<CS123SceneLightData>& sceneLights,
                             const Eigen::Vector3f& hitPos, const Eigen::Vector3f& hitNormal );

    Eigen::Vector3f uniformHemiSphere(float x, float y) ;
    void createNormalPlane(const Eigen::Vector3f & v1, Eigen::Vector3f & v2,Eigen::Vector3f & v3);
    std::vector<Triangle*> triangleLigthSources;


};

#endif // PATHTRACER_H
