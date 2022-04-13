//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <opencv2/opencv.hpp>

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
    Vector3f eye_pos(-1, 5, 10); // ��������ת�����λ�ã�-z��鿴
    int m = 0;
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
            // TODO: Find the x and y positions of the current pixel to get the
            // direction
            //  vector that passes through it.
            // Also, don't forget to multiply both of them with the variable
            // *scale*, and x (horizontal) variable with the *imageAspectRatio*

            x = i * 2.0 * scale * imageAspectRatio / scene.width;
            y = j * 2.0 * scale / scene.height;
            float dx = 2 * scale * imageAspectRatio / scene.width;
            float dy = 2 * scale / scene.height;
            x += (dx / 2);
            y += (dy / 2);
            x -= (2.0 * scale * imageAspectRatio / 2);
            y -= (2.0 * scale / 2);

            Vector3f dir = Vector3f(x, y, -1); // Don't forget to normalize this direction!
            dir = normalize(dir);
            int ind = (scene.height - 1 - j) * scene.width + i;
            
            Ray ray(eye_pos, dir);
            //Intersection inter = scene.bvh->Intersect(ray);

            framebuffer[ind] = scene.castRay(ray, 0) * 255.0f;

            //framebuffer[ind] = castRay(eye_pos, dir, scene, 0) * 255.0f;

            // Don't forget to normalize this direction!

        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    cv::Mat image(scene.height, scene.width, CV_32FC3, framebuffer.data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    cv::imwrite("test.jpg", image);

    // save framebuffer to file
    FILE* fp;
    fopen_s(&fp, "binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
