//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <future>
#include "Scene.hpp"
#include "Renderer.hpp"
#include "threadpool.hpp"
#include <tuple>


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    ThreadPool pool(1);

    // change the spp value to change sample ammount
    int spp = 2;
    std::cout << "SPP: " << spp << "\n";
    std::vector<std::future<std::tuple<int, Vector3f>>> results;
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            auto ray = Ray(eye_pos, dir);
            auto fn = [&scene](int m, Ray ray, int spp) {
                Vector3f result(0.f);
                for(int k = 0; k < spp; k++) {
                    result += scene.castRay(ray, 0)/spp;
                }
                return std::tuple<int, Vector3f>({m, result});
            };
            results.push_back(pool.enqueue(fn, m, ray, spp));
            m++;
        }
        // UpdateProgress(j / (float)scene.height);
    }
    for(int i = 0; i < results.size(); i++) {
        auto[m, res] = results[i].get();
        framebuffer[m] = res;
        UpdateProgress(i / (float)scene.height/scene.width);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
