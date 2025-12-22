namespace Perceptral {
namespace Component {
struct Renderable {
  bool castShadows = false;
  bool receiveShadows = false;
  int renderLayer = 0;

  Renderable() = default;
};
} // namespace Component
} // namespace Perceptral