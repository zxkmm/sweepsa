// A minimal OpenGL 3.3 core-profile entry point loader.
//
// Why this exists rather than glad or GLEW: the waterfall (§6) is the only part
// of sweepui that issues its own GL, and it needs about forty entry points.
// Dear ImGui's OpenGL backend carries its own loader, but that one is private to
// its translation unit and documented as internal, so it cannot be borrowed.
// Adding a whole loader library as a fourth submodule to call forty functions is
// a worse trade than forty lines of function pointers.
//
// Everything is loaded through glfwGetProcAddress, which matters on Windows:
// wglGetProcAddress returns null for the GL 1.1 functions that opengl32.dll
// exports directly, and GLFW handles that fallback. Hand-rolling the lookup with
// wglGetProcAddress alone would work on Linux and fail on Windows at
// glBindTexture, which §6 requires to build.
//
// Names live in sweepui::gl without the `gl` prefix -- gl::BindTexture, not
// glBindTexture -- so this header can be included in the same program as the
// system GL headers without either shadowing the other.

#pragma once

#include <cstddef>

namespace sweepui {
namespace gl {

// GL's typedefs, spelled in plain C types. The ABI is what matters here, and
// pulling in <GL/gl.h> for the aliases would drag in the platform's idea of
// which GL version exists, which on Windows is 1.1.
using Enum = unsigned int;
using Boolean = unsigned char;
using Bitfield = unsigned int;
using Int = int;
using Sizei = int;
using UInt = unsigned int;
using Float = float;
using Char = char;
using Intptr = std::ptrdiff_t;
using Sizeiptr = std::ptrdiff_t;

// Only the constants this program uses. Values are from the GL specification
// and are fixed for all time; the alternative is including a header that also
// declares the functions we are deliberately loading ourselves.
constexpr Enum kColorBufferBit = 0x00004000;
constexpr Enum kTexture2D = 0x0DE1;
constexpr Enum kTexture0 = 0x84C0;
constexpr Enum kTexture1 = 0x84C1;
constexpr Enum kNearest = 0x2600;
constexpr Enum kLinear = 0x2601;
constexpr Enum kTextureMagFilter = 0x2800;
constexpr Enum kTextureMinFilter = 0x2801;
constexpr Enum kTextureWrapS = 0x2802;
constexpr Enum kTextureWrapT = 0x2803;
constexpr Enum kClampToEdge = 0x812F;
constexpr Enum kRed = 0x1903;
constexpr Enum kRgb = 0x1907;
constexpr Enum kRgba = 0x1908;
constexpr Enum kR32F = 0x822E;
constexpr Enum kRgba8 = 0x8058;
constexpr Enum kFloat = 0x1406;
constexpr Enum kUnsignedByte = 0x1401;
constexpr Enum kFramebuffer = 0x8D40;
constexpr Enum kColorAttachment0 = 0x8CE0;
constexpr Enum kFramebufferComplete = 0x8CD5;
constexpr Enum kFramebufferBinding = 0x8CA6;
constexpr Enum kViewportState = 0x0BA2;
constexpr Enum kArrayBuffer = 0x8892;
constexpr Enum kStaticDraw = 0x88E4;
constexpr Enum kVertexShader = 0x8B31;
constexpr Enum kFragmentShader = 0x8B30;
constexpr Enum kCompileStatus = 0x8B81;
constexpr Enum kLinkStatus = 0x8B82;
constexpr Enum kFalse = 0;
constexpr Enum kTriangleStrip = 0x0005;
constexpr Enum kUnpackAlignment = 0x0CF5;
constexpr Enum kUnpackRowLength = 0x0CF2;
constexpr Enum kBlend = 0x0BE2;
constexpr Enum kScissorTest = 0x0C11;
constexpr Enum kDepthTest = 0x0B71;
constexpr Enum kCullFace = 0x0B44;
constexpr Enum kMaxTextureSize = 0x0D33;
constexpr Enum kPackAlignment = 0x0D05;
constexpr Enum kNoError = 0;

// The entry points, in one list so declaration and definition cannot drift.
#define SWEEPUI_GL_FUNCTIONS(X)                                                          \
    X(void, GenTextures, (Sizei, UInt *))                                                \
    X(void, DeleteTextures, (Sizei, const UInt *))                                       \
    X(void, BindTexture, (Enum, UInt))                                                   \
    X(void, ActiveTexture, (Enum))                                                       \
    X(void, TexImage2D, (Enum, Int, Int, Sizei, Sizei, Int, Enum, Enum, const void *))   \
    X(void, TexSubImage2D, (Enum, Int, Int, Int, Sizei, Sizei, Enum, Enum, const void *))\
    X(void, TexParameteri, (Enum, Enum, Int))                                            \
    X(void, PixelStorei, (Enum, Int))                                                    \
    X(void, GenFramebuffers, (Sizei, UInt *))                                            \
    X(void, DeleteFramebuffers, (Sizei, const UInt *))                                   \
    X(void, BindFramebuffer, (Enum, UInt))                                               \
    X(void, FramebufferTexture2D, (Enum, Enum, Enum, UInt, Int))                         \
    X(Enum, CheckFramebufferStatus, (Enum))                                              \
    X(UInt, CreateShader, (Enum))                                                        \
    X(void, DeleteShader, (UInt))                                                        \
    X(void, ShaderSource, (UInt, Sizei, const Char *const *, const Int *))               \
    X(void, CompileShader, (UInt))                                                       \
    X(void, GetShaderiv, (UInt, Enum, Int *))                                            \
    X(void, GetShaderInfoLog, (UInt, Sizei, Sizei *, Char *))                            \
    X(UInt, CreateProgram, (void))                                                       \
    X(void, DeleteProgram, (UInt))                                                       \
    X(void, AttachShader, (UInt, UInt))                                                  \
    X(void, LinkProgram, (UInt))                                                         \
    X(void, GetProgramiv, (UInt, Enum, Int *))                                           \
    X(void, GetProgramInfoLog, (UInt, Sizei, Sizei *, Char *))                           \
    X(void, UseProgram, (UInt))                                                          \
    X(Int, GetUniformLocation, (UInt, const Char *))                                     \
    X(void, Uniform1i, (Int, Int))                                                       \
    X(void, Uniform1f, (Int, Float))                                                     \
    X(void, Uniform2f, (Int, Float, Float))                                              \
    X(void, GenVertexArrays, (Sizei, UInt *))                                            \
    X(void, DeleteVertexArrays, (Sizei, const UInt *))                                   \
    X(void, BindVertexArray, (UInt))                                                     \
    X(void, GenBuffers, (Sizei, UInt *))                                                 \
    X(void, DeleteBuffers, (Sizei, const UInt *))                                        \
    X(void, BindBuffer, (Enum, UInt))                                                    \
    X(void, BufferData, (Enum, Sizeiptr, const void *, Enum))                            \
    X(void, EnableVertexAttribArray, (UInt))                                             \
    X(void, VertexAttribPointer, (UInt, Int, Enum, Boolean, Sizei, const void *))        \
    X(void, DrawArrays, (Enum, Int, Sizei))                                              \
    X(void, Viewport, (Int, Int, Sizei, Sizei))                                          \
    X(void, ClearColor, (Float, Float, Float, Float))                                    \
    X(void, Clear, (Bitfield))                                                           \
    X(void, Disable, (Enum))                                                             \
    X(void, GetIntegerv, (Enum, Int *))                                                  \
    X(void, ReadPixels, (Int, Int, Sizei, Sizei, Enum, Enum, void *))                    \
    X(Enum, GetError, (void))

#define SWEEPUI_GL_DECLARE(ret, name, args) extern ret(*name) args;
SWEEPUI_GL_FUNCTIONS(SWEEPUI_GL_DECLARE)
#undef SWEEPUI_GL_DECLARE

// Loads every entry point above. False, with the first missing name in *err, if
// the context does not provide one -- which is how a machine that opened a
// context older than 3.3 finds out at startup instead of at the first null call.
bool load(void *(*get_proc)(const char *), const char **missing);

}  // namespace gl
}  // namespace sweepui
