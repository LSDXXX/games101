//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE, MICROFACET};

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

    float GGX_D(float roughness, float nwm)  {
        /*
        auto roughness2 = roughness*roughness;
        auto nwm2 = nwm*nwm;
        auto tmp = nwm2*(roughness2-1)+1;
        return roughness2/(M_PI*tmp*tmp);
        */
        auto roughness2 = roughness*roughness;
        auto nwm2 = nwm*nwm;
        float tmp = (nwm2-1) / (roughness2*nwm2);
        return std::exp(tmp) / (roughness2*std::pow(nwm, 4.0));
    }

    float Smith_G(float roughness, float ni, float nr, float nh, float im) {
        auto G1 = 2*nh*ni/im;
        auto G2 = 2*nh*nr/im;
        return  std::min(1.0f, std::min(G1, G2));
    }

    Vector3f eval_diffuse(const Vector3f& N, const Vector3f& wo) {
        float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
    }

    float eval_microfacet(
    const Vector3f &wi, const Vector3f &wo, const Vector3f &N,
    const float &ior, const float &roughness
){

    float F = 0.f;
    fresnel(wi, N, ior, F);

    auto wm = normalize(wi+wo);
    
    auto D = GGX_D(roughness, dotProduct(N, wm));

    auto G = Smith_G(roughness, dotProduct(wi, N), dotProduct(wo, N), dotProduct(N, wm), dotProduct(wm, wi));

    auto fr = F*G*D / 4/ std::max(EPSILON, dotProduct(N,wo)*dotProduct(N,wi));
    /*
    float eps = 1e-6;
    Vector3f h = normalize(wi + wo);
    float N_dot_wo = std::max(0.0f, dotProduct(wo, N));
    float N_dot_wi = std::max(0.0f, dotProduct(wi, N));
    float N_dot_h = std::max(0.0f, dotProduct(h, N));
    float m = roughness; //rms slope, roughness
    // fresnel term, exact form
    float F;
    fresnel(wi, N, ior, F);
    // shadow masking term
    float G = 1;
    float G1 = 2 * N_dot_h * N_dot_wo
                    / std::max(0.0f, dotProduct(wo, h));
    float G2 = 2 * N_dot_h * N_dot_wi
                    / std::max(0.0f, dotProduct(wo, h));
    G = std::min({G, G1, G2});

    // G = std::max(G, 0.0f);
    // normal distribution, beckman distribution
    float D;
    float m2 = m*m;
    float chi_Nh = (float)(N_dot_h > 0);
    float alpha = acos(N_dot_h);
    float tan_alpha = tan(alpha);
    float cos_alpha = cos(alpha);
    float cos_alpha2 = cos_alpha*cos_alpha;
    float cos_alpha4 = cos_alpha2*cos_alpha2;
    D = chi_Nh * exp(-tan_alpha*tan_alpha / m2) 
                / M_PI / m2 / cos_alpha4;

    // float ker_dist = N_dot_h / m;
    // D = exp(-ker_dist*ker_dist);

    // USER_NOTE:
    // use the cook-torrance formula on wiki, page: Specular_highlight
    // float fr = (F*G*D) / 4 / std::max(eps, N_dot_wi*N_dot_wo);
    std::cout << "F: " << F << "G " << G << "D " << D << std::endl;
    float fr = (F*G*D) / 4/ std::max(eps, N_dot_wi*N_dot_wo);
    // std::clog << fr << std::endl;
    */
    return fr;
}

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;
    float roughness;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
    roughness = 0.8;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
        case MICROFACET:
        {
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }

    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else {
                return 0.0f;
            }
            break;
        }
        case MICROFACET:
        {
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else {
                return 0.0f;
            }
            break;
        }

    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    /*
        auto wm = normalize(wo-wi);
        auto nwi = dotProduct(N, wi), nwo = dotProduct(N, wo),
        nwm = dotProduct(N, wm);
    
        auto D = this->GGX_D(this->roughness, nwm);
        auto G = this->Smith_G(this->roughness, nwi, nwo);
        float F = 0.f;
        this->fresnel(wi, N, this->ior, F);

        auto res = F*std::max(D*G/(4*std::fabs(nwi)*std::fabs(nwo)), 0.f);
        return res;
        */

    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse   model
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case MICROFACET: 
        {
            // impl microfacet
            /* 
            auto i = wi;
            auto r = wo;
            auto m = normalize(r + i);
            auto ni = dotProduct(N, i), nr = dotProduct(N, r),
            nm = dotProduct(N, m), im = dotProduct(i,m);
            float res = 0.f;
    
            if(ni*nr > 100*EPSILON) {
                auto D = this->GGX_D(this->roughness, nm);
                auto G = this->Smith_G(this->roughness, ni, nr, nm, im);
                float F = 0.f;
                this->fresnel(i, N, this->ior, F);

                res = F*F*D*G/(4*ni*nr);
            }
            if(res > 1.f) 
             {
            //      std::cout << res << " F: " << F << " D:" << D 
            //  << " G:" << G << " nwm: " << nwm << " nwi: " << nwi << " nwo: " << nwo << std::endl;
                res = 1.f;
             }

            // std::cout << res << std::endl;
            // std::cout << "wm:" << wm << std::endl;
            // std::cout << "wi:" << wi << std::endl;
            // std::cout << "wo:" << wo << std::endl;
            // std::cout << "N:" << N << std::endl;
            return res;
            */
           auto fr =eval_microfacet(wi, wo, N, this->ior, this->roughness);
           auto fd = eval_diffuse(N, wo);
           
           if(!isValidF(fr)) std::cout << fr << std::endl;
           return 0.7*fd + 0.3* fr;
        }
    }
}

#endif //RAYTRACING_MATERIAL_H
