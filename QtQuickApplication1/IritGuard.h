#pragma once
//
// IritGuard - turns IRIT's fatal errors into a normal false return.
//
// IRIT's default fatal handlers print to stderr and call exit(). In a GUI
// app that means a malformed file makes the whole window vanish with no
// message. We install our own handlers that record the message and longjmp
// back out of the library.
//
// Because longjmp cannot safely unwind C++ frames that own destructors,
// run() takes a plain function pointer plus a void* context. Put whatever
// the callback needs in a POD struct and keep C++ objects outside.
//
#include <QString>

namespace IritGuard {

// Install the fatal-error handlers on every IRIT library we link. Call once,
// from main(), before any other IRIT call.
void installHandlers();

// Invoke fn(ctx) with IRIT fatal errors trapped.
// Returns true if fn ran to completion, false if IRIT raised a fatal error.
bool run(void *ctx, void (*fn)(void *ctx));

// Message from the most recent trapped error (empty if none).
QString lastError();

} // namespace IritGuard
