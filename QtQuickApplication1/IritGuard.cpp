#include "IritGuard.h"

#include <csetjmp>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#ifdef _MSC_VER
#include <crtdbg.h>
#endif

extern "C" {
#include "inc_irit/irit_sm.h"
#include "inc_irit/misc_lib.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/cagd_lib.h"
#include "inc_irit/symb_lib.h"
#include "inc_irit/trim_lib.h"
#include "inc_irit/triv_lib.h"
#include "inc_irit/trng_lib.h"
#include "inc_irit/mdl_lib.h"
#include "inc_irit/mvar_lib.h"
#include "inc_irit/bool_lib.h"
#include "inc_irit/geom_lib.h"
}

// MSVC warns that setjmp/longjmp bypasses destructor calls. That is exactly
// why run() is restricted to a POD context and a plain function pointer.
#pragma warning(disable : 4611)

namespace {

jmp_buf g_jmp;
bool    g_armed = false;
char    g_msg[512] = { 0 };

void record(const char *lib, const char *desc)
{
    std::snprintf(g_msg, sizeof g_msg, "IRIT %s: %s",
                  lib  ? lib  : "?",
                  desc ? desc : "unspecified fatal error");
}

void bailOut()
{
    if (g_armed) {
        g_armed = false;
        std::longjmp(g_jmp, 1);
    }
    // Nothing armed: we are outside a guarded call. Returning lets IRIT
    // continue its own error path rather than jumping into a dead frame.
}

// One thunk per library. Each maps the library's error enum to text via that
// library's DescribeError, then bails out.
void onMisc(const char *m)              { record("misc", m);                                   bailOut(); }
void onPrsr(IritPrsrFatalErrorType e)   { record("prsr", IritPrsrDescribeError(e));            bailOut(); }
void onCagd(IritCagdFatalErrorType e)   { record("cagd", IritCagdDescribeError(e));            bailOut(); }
void onSymb(IritSymbFatalErrorType e)   { record("symb", IritSymbDescribeError(e));            bailOut(); }
void onTrim(IritTrimFatalErrorType e)   { record("trim", IritTrimDescribeError(e));            bailOut(); }
void onTriv(IritTrivFatalErrorType e)   { record("triv", IritTrivDescribeError(e));            bailOut(); }
void onTrng(IritTrngFatalErrorType e)   { record("trng", IritTrngDescribeError(e));            bailOut(); }
void onMdl (IritMdlFatalErrorType  e)   { record("mdl",  IritMdlDescribeError(e));             bailOut(); }
void onMvar(IritMvarFatalErrorType e)   { record("mvar", IritMvarDescribeError(e));            bailOut(); }
void onBool(BoolFatalErrorType     e)   { record("bool", IritBoolDescribeError(e));            bailOut(); }
void onGeom(IritGeomFatalErrorType e)   { record("geom", IritGeomDescribeError(e));            bailOut(); }

// Not every IRIT failure goes through a fatal-error callback. The IRIT
// libraries linked here are built with DEBUG, so some unhandled geometric
// configurations end in assert() -> abort() instead: a boolean on a spiky
// non-convex model dumps the intersection loop it could not close and the
// process dies with exit code 3, taking every other piece down with it.
//
// abort() cannot be trapped by the callbacks above, but it does raise SIGABRT
// first, and that can be. Jumping out of the handler leaves IRIT's own heap
// state unknown, so this is containment, not recovery: it exists so one
// impossible piece is reported and skipped rather than ending the run.
extern "C" void onAbort(int)
{
    record("bool", "assertion failed inside IRIT (unsupported configuration)");
    bailOut();
}

} // namespace

void IritGuard::installHandlers()
{
    IritMiscSetFatalErrorFunc(onMisc);
    IritPrsrSetFatalErrorFunc(onPrsr);
    IritCagdSetFatalErrorFunc(onCagd);
    IritSymbSetFatalErrorFunc(onSymb);
    IritTrimSetFatalErrorFunc(onTrim);
    IritTrivSetFatalErrorFunc(onTriv);
    IritTrngSetFatalErrorFunc(onTrng);
    IritMdlSetFatalErrorFunc(onMdl);
    IritMvarSetFatalErrorFunc(onMvar);
    IritBoolSetFatalErrorFunc(onBool);
    IritGeomSetFatalErrorFunc(onGeom);
}

bool IritGuard::run(void *ctx, void (*fn)(void *))
{
    g_msg[0] = '\0';

    if (setjmp(g_jmp) != 0) {
        // Back here from a fatal error or an assertion. The CRT report mode is
        // deliberately left as-is: another guarded call is usually next, and a
        // dialog appearing between them is exactly what this avoids.
        g_armed = false;
        std::signal(SIGABRT, SIG_DFL);
        return false;
    }

    // Trap the abort path for the duration of the call only, and put the
    // previous disposition back afterwards so nothing outside is affected.
#ifdef _MSC_VER
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
    // Send IRIT's assertions to stderr rather than to a message box. The debug
    // CRT puts up a modal Abort/Retry/Ignore dialog BEFORE it raises SIGABRT,
    // so without this the window blocks the whole app and the handler below
    // never runs - invisible in a console run, fatal in the GUI.
#if defined(_MSC_VER) && defined(_DEBUG)
    const int prevReport = _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
#endif
    void (*prevAbort)(int) = std::signal(SIGABRT, onAbort);

    g_armed = true;
    fn(ctx);
    g_armed = false;

    std::signal(SIGABRT, prevAbort);
#if defined(_MSC_VER) && defined(_DEBUG)
    _CrtSetReportMode(_CRT_ASSERT, prevReport);
#endif
    return true;
}

QString IritGuard::lastError()
{
    return QString::fromLocal8Bit(g_msg);
}
