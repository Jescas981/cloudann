#include "renderer/ShaderLibrary.h"
#include <iostream>

namespace CloudCore {

void ShaderLibrary::add(const std::string& name, const std::shared_ptr<Shader>& shader) {
    if (exists(name)) {
        std::cerr << "Shader already exists: " << name << std::endl;
        return;
    }
    shaders_[name] = shader;
}

void ShaderLibrary::add(const std::shared_ptr<Shader>& shader) {
    auto& name = shader->getName();
    add(name, shader);
}

std::shared_ptr<Shader> ShaderLibrary::load(const std::string& filepath) {
    auto shader = Shader::create(filepath);
    add(shader);
    return shader;
}

std::shared_ptr<Shader> ShaderLibrary::load(const std::string& name, const std::string& filepath) {
    auto shader = Shader::create(filepath);
    add(name, shader);
    return shader;
}

std::shared_ptr<Shader> ShaderLibrary::get(const std::string& name) {
    if (!exists(name)) {
        std::cerr << "Shader not found: " << name << std::endl;
        return nullptr;
    }
    return shaders_[name];
}

bool ShaderLibrary::exists(const std::string& name) const {
    return shaders_.find(name) != shaders_.end();
}

} // namespace CloudCore
