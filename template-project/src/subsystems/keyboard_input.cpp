#include "keyboard_input.hpp"

namespace src
{
// Static member definitions
bool KeyboardInput::keyStates[256] = {false};
float KeyboardInput::mouseX = 0.0f;
float KeyboardInput::mouseY = 0.0f;
bool KeyboardInput::leftMouseClick = false;
bool KeyboardInput::rightMouseClick = false;
float KeyboardInput::mouseWheelDelta = 0.0f;

bool KeyboardInput::isKeyPressed(char key)
{
    return keyStates[static_cast<unsigned char>(key)];
}

void KeyboardInput::setKeyState(char key, bool pressed)
{
    keyStates[static_cast<unsigned char>(key)] = pressed;
}

float KeyboardInput::getMouseX()
{
    return mouseX;
}

float KeyboardInput::getMouseY()
{
    return mouseY;
}

void KeyboardInput::setMousePosition(float x, float y)
{
    mouseX = x;
    mouseY = y;
}

bool KeyboardInput::isLeftMouseClicked()
{
    return leftMouseClick;
}

bool KeyboardInput::isRightMouseClicked()
{
    return rightMouseClick;
}

float KeyboardInput::getMouseWheelDelta()
{
    return mouseWheelDelta;
}

void KeyboardInput::setLeftMouseClick(bool clicked)
{
    leftMouseClick = clicked;
}

void KeyboardInput::setRightMouseClick(bool clicked)
{
    rightMouseClick = clicked;
}

void KeyboardInput::setMouseWheelDelta(float delta)
{
    mouseWheelDelta = delta;
}

void KeyboardInput::updateFrame()
{
    // Reset click states and wheel delta at end of frame
    leftMouseClick = false;
    rightMouseClick = false;
    mouseWheelDelta = 0.0f;
}
}  // namespace src
