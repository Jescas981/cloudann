#include <Perceptral/core/DeltaTime.h>
#include <Perceptral/core/Log.h>
#include <Perceptral/scene/Scriptable.h>

class CubeScriptable : public Perceptral::Scriptable {
public:
  void onCreate() override {}

  void onUpdate(Perceptral::DeltaTime dt) override {}

  void onDestroy() override {}
};