#pragma once
//
// IritGuard - turns IRIT's fatal errors into an ordinary false return.
//
// IRIT's default handlers print to stderr and call exit(), which in a GUI means a
// malformed file makes the window vanish with no message. These record the
// message and longjmp back out of the library instead.
//
// longjmp cannot safely unwind C++ frames that own destructors, so run() takes a
// plain function pointer and a void* context. Keep C++ objects outside it.
//
#include <QString>

namespace IritGuard {

void installHandlers();

bool run(void *ctx, void (*fn)(void *ctx));

QString lastError();

}
