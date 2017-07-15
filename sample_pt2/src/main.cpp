//-------------------------------------------------------------------------------------------------
// File : main.cpp
// Desc : Main Entry Point.
// Copyright(c) Project Asura. All right reserved.
//-------------------------------------------------------------------------------------------------

#define ENABLE_NEXT_EVENT_ESTIMATION (1)

//-------------------------------------------------------------------------------------------------
// Inlcudes
//-------------------------------------------------------------------------------------------------
#include <r3d_math.h>
#include <r3d_camera.h>
#include <r3d_shape.h>
#include <vector>
#include <stb/stb_image_write.h>
#include <stb/stb_image.h>
#include <algorithm>

namespace {

//-------------------------------------------------------------------------------------------------
// Global Varaibles.
//-------------------------------------------------------------------------------------------------
const int     g_max_depth = 5;
const Vector3 g_back_ground (0.0,   0.0,    0.0);
const Sphere  g_spheres[] = {
    Sphere(1e5,     Vector3( 1e5 + 1.0,    40.8,          81.6), Vector3(0.25,  0.75,  0.25), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(-1e5 + 99.0,   40.8,          81.6), Vector3(0.25,  0.25,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,          40.8,           1e5), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,          40.8,  -1e5 + 170.0), Vector3(0.01,  0.01,  0.01), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,           1e5,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(1e5,     Vector3(50.0,   -1e5 + 81.6,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse,          Vector3(0, 0, 0)),
    Sphere(16.5,    Vector3(27.0,          16.5,          47.0), Vector3(0.75,  0.25,  0.25), ReflectionType::PerfectSpecular,  Vector3(0, 0, 0)),
    Sphere(16.5,    Vector3(73.0,          16.5,          78.0), Vector3(0.99,  0.99,  0.99), ReflectionType::Refraction,       Vector3(0, 0, 0)),
    Sphere(5.0,     Vector3(50.0,          81.6,          81.6), Vector3(),                   ReflectionType::Diffuse,          Vector3(12, 12, 12))
};
const int   g_lightId = 8;


//-------------------------------------------------------------------------------------------------
//      シーンとの交差判定を行います.
//-------------------------------------------------------------------------------------------------
inline bool intersect_scene(const Ray& ray, double* t, int* id)
{
    auto n = static_cast<int>(sizeof(g_spheres) / sizeof(g_spheres[0]));

    *t  = D_MAX;
    *id = -1;

    for (auto i = 0; i < n; ++i)
    {
        auto d = g_spheres[i].intersect(ray);
        if (d > D_HIT_MIN && d < *t)
        {
            *t  = d;
            *id = i;
        }
    }

    return (*t < D_HIT_MAX);
}

//-------------------------------------------------------------------------------------------------
//      放射輝度を求めます.
//-------------------------------------------------------------------------------------------------
Vector3 radiance(const Ray& input_ray, Random* random)
{
    Vector3 L(0, 0, 0);
    Vector3 W(1, 1, 1);
    Ray ray(input_ray.pos, input_ray.dir);

    #if ENABLE_NEXT_EVENT_ESTIMATION
    auto direct_light = true;
    #endif

    for(int depth=0; ; depth++)
    {
        double t;
        int   id;

        // シーンとの交差判定.
        if (!intersect_scene(ray, &t, &id))
        { break; }

        // 交差物体.
        const auto& obj = g_spheres[id];

        // 交差位置.
        const auto hit_pos = ray.pos + ray.dir * t;

        // 法線ベクトル.
        const auto normal  = normalize(hit_pos - obj.pos);

        // 物体からのレイの入出を考慮した法線ベクトル.
        const auto orienting_normal = (dot(normal, ray.dir) < 0.0) ? normal : -normal;

        auto p = max(obj.color.x, max(obj.color.y, obj.color.z));

        #if ENABLE_NEXT_EVENT_ESTIMATION
        {
            if (direct_light)
            { L += W * obj.emission; }

            direct_light = (obj.type != Diffuse);
        }
        #else
        {
            L += W * obj.emission;
        }
        #endif


        // 打ち切り深度に達したら終わり.
        if(depth > g_max_depth)
        {
            if (random->get_as_double() >= p)
            { break; }
        }
        else
        {
            p = 1.0;
        }

        switch (obj.type)
        {
        case ReflectionType::Diffuse:
            {
                #if ENABLE_NEXT_EVENT_ESTIMATION
                // Next Event Estimation
                {
                    const auto& light = g_spheres[g_lightId];

                    const auto r1 = D_2PI * random->get_as_double();
                    const auto r2 = 1.0 - 2.0 * random->get_as_double();
                    const auto light_pos = light.pos + (light.radius + D_HIT_MIN) * Vector3(sqrt(1.0 - r2 * r2) * cos(r1), sqrt(1.0 - r2 * r2) * sin(r1), r2);

                    // ライトベクトル.
                    auto light_dir   = light_pos - hit_pos;

                    // ライトへの距離の2乗
                    auto light_dist2 = dot(light_dir, light_dir);

                    // 正規化.
                    light_dir = normalize(light_dir);

                    // ライトの法線ベクトル.
                    auto light_normal = normalize(light_pos - light.pos);

                    auto dot0 = dot(orienting_normal, light_dir);
                    auto dot1 = dot(light_normal, -light_dir);
                    auto rad2 = light.radius * light.radius;

                    // 寄与が取れる場合.
                    if (dot0 >= 0 && dot1 >= 0)
                    {
                        double shadow_t;
                        int    shadow_id;
                        Ray    shadow_ray(hit_pos, light_dir);

                        // シャドウレイを発射.
                        auto hit = intersect_scene(shadow_ray, &shadow_t, &shadow_id);

                        // ライトのみと衝突した場合のみ寄与を取る.
                        if (hit && shadow_id == g_lightId)
                        {
                            auto G = dot0 * dot1 / light_dist2;
                            auto pdf = 1.0 / (4.0 * D_PI * rad2);

                            L += W * light.emission * (obj.color / D_PI) * G / pdf;
                        }
                    }
                }
                #endif

                // 基底ベクトル.
                Vector3 u, v, w;

                w = orienting_normal;
                if (abs(w.x) > 0.1)
                { u = normalize(cross(Vector3(0, 1, 0), w)); }
                else
                { u = normalize(cross(Vector3(1, 0, 0), w)); }
                v = cross(w, u);

                const auto r1 = D_2PI * random->get_as_double();
                const auto r2 = random->get_as_double();
                const auto r2s = sqrt(r2);

                auto dir = normalize(u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1.0 - r2));

                ray = Ray(hit_pos, dir);
                W *= (obj.color / p);
            }
            break;

        case ReflectionType::PerfectSpecular:
            {
                ray = Ray(hit_pos, reflect(ray.dir, normal));
                W *= (obj.color / p);
            }
            break;

        case ReflectionType::Refraction:
            {
                Ray reflect_ray = Ray(hit_pos, reflect(ray.dir, normal));
                auto into = dot(normal, orienting_normal) > 0.0;

                const auto nc = 1.0;
                const auto nt = 1.5;
                const auto nnt = (into) ? (nc / nt) : (nt / nc);
                const auto ddn = dot(ray.dir, orienting_normal);
                const auto cos2t = 1.0 - nnt * nnt * (1.0 - ddn * ddn);

                if (cos2t <= 0.0)
                {
                    ray = reflect_ray;
                    W *= (obj.color / p);
                    break;
                }

                auto dir = normalize(ray.dir * nnt - normal * ((into) ? 1.0 : -1.0) * (ddn * nnt + sqrt(cos2t)));

                const auto a = nt - nc;
                const auto b = nt + nc;
                const auto R0 = (a * a) / (b * b);
                const auto c = 1.0 - ((into) ? -ddn : dot(dir, normal));
                const auto Re = R0 + (1.0 - R0) * pow(c, 5.0);
                const auto Tr = 1.0 - Re;
                const auto prob = 0.25 + 0.5 * Re;

                if (random->get_as_double() < prob)
                {
                    ray = reflect_ray;
                    W *= (obj.color * Re / prob) / p; 
                }
                else
                {
                    ray = Ray(hit_pos, dir);
                    W *= (obj.color * Tr / (1.0 - prob)) / p;
                }
            }
            break;
        }
    }

    return L;
}

//-------------------------------------------------------------------------------------------------
//      BMPファイルに保存します.
//-------------------------------------------------------------------------------------------------
void save_to_bmp(const char* filename, int width, int height, const double* pixels)
{
    std::vector<uint8_t> images;
    images.resize(width * height * 3);

    const double inv_gamma = 1.0 / 2.2;

    for(auto i=0; i<width * height * 3; i+=3)
    {
        auto r = pow(pixels[i + 0], inv_gamma);
        auto g = pow(pixels[i + 1], inv_gamma);
        auto b = pow(pixels[i + 2], inv_gamma);

        r = saturate(r);
        g = saturate(g);
        b = saturate(b);

        images[i + 0] = static_cast<uint8_t>( r * 255.0 + 0.5 );
        images[i + 1] = static_cast<uint8_t>( g * 255.0 + 0.5 );
        images[i + 2] = static_cast<uint8_t>( b * 255.0 + 0.5 );
    }

    stbi_write_bmp(filename, width, height, 3, images.data());
}

//-------------------------------------------------------------------------------------------------
//      中央値フィルタを適用します.
//-------------------------------------------------------------------------------------------------
void median_filter(int w, int h, std::vector<Vector3>& pixels)
{
    // BT.601 での輝度値への変換.
    Vector3 to_lum(0.299f, 0.587f, 0.114f);

    struct item
    {
        double luminance;
        int    array_index;

        item()
        { /* DO_NOTHING */ }

        item(double lum, int index)
            : luminance(lum)
            , array_index(index)
        { /* DO_NOTHING */ }

        bool operator < (const item& value)
        { return luminance < value.luminance; }
    };

    for (auto y = 0; y < h; ++y)
    {
        for (auto x = 0; x < w; ++x)
        {
            Vector3 px[9] = {};
            px[4] = pixels[y * w + x]; // 中央
            px[0] = (x == 0 || y == 0)               ? px[4] : pixels[(y - 1) * w + (x - 1)];     // 左上
            px[1] = (y == 0)                         ? px[4] : pixels[(y - 1) * w + (x + 0)];     // 上
            px[2] = (x == (w - 1) || y == 0)         ? px[4] : pixels[(y - 1) * w + (x + 1)];     // 右上
            px[3] = (x == 0)                         ? px[4] : pixels[(y + 0) * w + (x - 1)];     // 左
            px[5] = (x == (w - 1))                   ? px[4] : pixels[(y + 0) * w + (x + 1)];     // 右
            px[6] = (x == 0 || y == (h - 1))         ? px[4] : pixels[(y + 1) * w + (x - 1)];     // 左下
            px[7] = (y == (h - 1))                   ? px[4] : pixels[(y + 1) * w + (x + 0)];     // 下
            px[8] = (x == (w - 1) || y == (h - 1))   ? px[4] : pixels[(y + 1) * w + (x + 1)];     //　右下

            // 輝度値を求める.
            item lum[9];
            lum[0] = item(dot(px[0], to_lum), 0);
            lum[1] = item(dot(px[1], to_lum), 1);
            lum[2] = item(dot(px[2], to_lum), 2);
            lum[3] = item(dot(px[3], to_lum), 3);
            lum[4] = item(dot(px[4], to_lum), 4);
            lum[5] = item(dot(px[5], to_lum), 5);
            lum[6] = item(dot(px[6], to_lum), 6);
            lum[7] = item(dot(px[7], to_lum), 7);
            lum[8] = item(dot(px[8], to_lum), 8);

            // 並び替え.
            std::sort(std::begin(lum), std::end(lum));

            // 中央の値を採用.
            pixels[y * w + x] = px[lum[4].array_index];
        }
    }
}

} // namespace


//-------------------------------------------------------------------------------------------------
//      メインエントリーポイントです.
//-------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    // レンダーターゲットのサイズ.
    int width   = 640;
    int height  = 480;
    int samples = 512;

    // カメラ用意.
    Camera camera(
        Vector3(50.0, 52.0, 295.6),
        normalize(Vector3(0.0, -0.042612, -1.0)),
        Vector3(0.0, 1.0, 0.0),
        0.5135,
        double(width) / double(height),
        130.0
    );

    // レンダーターゲット生成.
    std::vector<Vector3> image;
    image.resize(width * height);

    Random random(123456);

    // レンダーターゲットをクリア.
    for (size_t i = 0; i < image.size(); ++i)
    { image[i] = g_back_ground; }

    for(auto s = 0; s < samples; ++s)
    {
        printf_s("%.2lf%% complete\r", (double(s)/double(samples) * 100.0));

        #pragma omp parallel for schedule(dynamic, 1) num_threads(4)
        for (auto y = 0; y < height; ++y)
        {
            for (auto x = 0; x < width; ++x)
            {   
                auto idx = y * width + x;

                auto fx = double(x) / double(width)  - 0.5;
                auto fy = double(y) / double(height) - 0.5;

                // Let's レイトレ！
                image[idx] += radiance(camera.emit(fx, fy), &random) / samples;
            }
        }
    }

    // 中央値フィルタを適用
    median_filter(width, height, image);

    // レンダーターゲットの内容をファイルに保存.
    save_to_bmp("image.bmp", width, height, &image.data()->x);

    // レンダーターゲットクリア.
    image.clear();

    return 0;
}