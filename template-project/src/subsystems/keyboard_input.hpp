#pragma once

namespace src
{
class KeyboardInput
{
public:
    // Keyboard state tracking
    static bool isKeyPressed(char key);
    static void setKeyState(char key, bool pressed);
    
    // Mouse tracking
    static float getMouseX();
    static float getMouseY();
    static void setMousePosition(float x, float y);
    
    static bool isLeftMouseClicked();
    static bool isRightMouseClicked();
    static float getMouseWheelDelta();
    
    static void setLeftMouseClick(bool clicked);
    static void setRightMouseClick(bool clicked);
    static void setMouseWheelDelta(float delta);
    
    // Reset states each frame
    static void updateFrame();

private:
    static bool keyStates[256];
    static float mouseX, mouseY;
    static bool leftMouseClick, rightMouseClick;
    static float mouseWheelDelta;
};
}  // namespace src
