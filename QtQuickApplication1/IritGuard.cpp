#include "IritGuard.h"

#include <csetjmp>
#include <cstdio>
#include <cstring>

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
        g_armed = false;
        return false;                       // trapped a fatal error
    }

    g_armed = true;
    fn(ctx);
    g_armed = false;
    return true;
}

QString IritGuard::lastError()
{
    return QString::fromLocal8Bit(g_msg);
}
