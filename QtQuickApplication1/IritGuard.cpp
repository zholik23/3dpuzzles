//
// IritGuard - implementation: one fatal handler per IRIT library, each recording
// the message and longjmp-ing back out of the library.
//

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

// Jumps back out of the library. Nothing armed means we are outside a guarded
// call, so IRIT is left to its own error path rather than jumping into a dead
// frame.
void bailOut()
{
    if (g_armed) {
        g_armed = false;
        std::longjmp(g_jmp, 1);
    }
}

// One thunk per IRIT library: map that library's error enum to text through its
// own DescribeError, then bail out.
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

extern "C" void onAbort(int)
{
    record("bool", "assertion failed inside IRIT (unsupported configuration)");
    bailOut();
}

}

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

// Runs fn(ctx) with IRIT's fatal errors and its assertions trapped. Containment,
// not recovery: after a longjmp IRIT's heap state is unknown, so one impossible
// piece is reported and skipped rather than ending the run.
bool IritGuard::run(void *ctx, void (*fn)(void *))
{
    g_msg[0] = '\0';

    if (setjmp(g_jmp) != 0) {
        g_armed = false;
        std::signal(SIGABRT, SIG_DFL);
        return false;
    }

#ifdef _MSC_VER
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
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
