#include "gl.h"

namespace sweepui {
namespace gl {

#define SWEEPUI_GL_DEFINE(ret, name, args) ret (*name) args = nullptr;
SWEEPUI_GL_FUNCTIONS(SWEEPUI_GL_DEFINE)
#undef SWEEPUI_GL_DEFINE

bool load(void *(*get_proc)(const char *), const char **missing) {
    if (!get_proc) {
        *missing = "(no loader)";
        return false;
    }

    // The GL name is the namespaced name with the `gl` prefix put back.
#define SWEEPUI_GL_LOAD(ret, name, args)                                    \
    do {                                                                    \
        name = (ret(*) args)get_proc("gl" #name);                           \
        if (!name) {                                                        \
            *missing = "gl" #name;                                          \
            return false;                                                   \
        }                                                                   \
    } while (0);
    SWEEPUI_GL_FUNCTIONS(SWEEPUI_GL_LOAD)
#undef SWEEPUI_GL_LOAD

    return true;
}

}  // namespace gl
}  // namespace sweepui
