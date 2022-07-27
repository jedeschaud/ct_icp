#include "SlamCore/utils.h"

#if !defined(_WIN32) && !defined(APPLE)
#define _IS_UNIX
#endif

#ifdef _IS_UNIX

/*
 * Code copied and adapted from https://gist.github.com/jvranish/4441299
 */

#define MAX_STACK_FRAMES 64

#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <err.h>
#include <execinfo.h>

// Returns the address line from a symbol by calling the command addr2line
int _get_adress_line(char const *const program_name, void const *const addr) {
    char addr2line_cmd[512] = {0};
    sprintf(addr2line_cmd, "addr2line -f -p -e %.256s %p", program_name, addr);
    return system(addr2line_cmd);
}

// Handles posix signals (detailing the signal caught + printing the stack trace)
void _posix_signal_handler(int sig, siginfo_t *siginfo, void *context) {
    (void) context;
    switch (sig) {
        case SIGSEGV:
            fputs("Caught SIGSEGV: Segmentation Fault\n", stderr);
            break;
        case SIGINT:
            fputs("Caught SIGINT: Interactive attention signal, (usually ctrl+c)\n", stderr);
            break;
        case SIGFPE:
            switch (siginfo->si_code) {
                case FPE_INTDIV:
                    fputs("Caught SIGFPE: (integer divide by zero)\n", stderr);
                    break;
                case FPE_INTOVF:
                    fputs("Caught SIGFPE: (integer overflow)\n", stderr);
                    break;
                case FPE_FLTDIV:
                    fputs("Caught SIGFPE: (floating-point divide by zero)\n", stderr);
                    break;
                case FPE_FLTOVF:
                    fputs("Caught SIGFPE: (floating-point overflow)\n", stderr);
                    break;
                case FPE_FLTUND:
                    fputs("Caught SIGFPE: (floating-point underflow)\n", stderr);
                    break;
                case FPE_FLTRES:
                    fputs("Caught SIGFPE: (floating-point inexact result)\n", stderr);
                    break;
                case FPE_FLTINV:
                    fputs("Caught SIGFPE: (floating-point invalid operation)\n", stderr);
                    break;
                case FPE_FLTSUB:
                    fputs("Caught SIGFPE: (subscript out of range)\n", stderr);
                    break;
                default:
                    fputs("Caught SIGFPE: Arithmetic Exception\n", stderr);
                    break;
            }
        case SIGILL:
            switch (siginfo->si_code) {
                case ILL_ILLOPC:
                    fputs("Caught SIGILL: (illegal opcode)\n", stderr);
                    break;
                case ILL_ILLOPN:
                    fputs("Caught SIGILL: (illegal operand)\n", stderr);
                    break;
                case ILL_ILLADR:
                    fputs("Caught SIGILL: (illegal addressing mode)\n", stderr);
                    break;
                case ILL_ILLTRP:
                    fputs("Caught SIGILL: (illegal trap)\n", stderr);
                    break;
                case ILL_PRVOPC:
                    fputs("Caught SIGILL: (privileged opcode)\n", stderr);
                    break;
                case ILL_PRVREG:
                    fputs("Caught SIGILL: (privileged register)\n", stderr);
                    break;
                case ILL_COPROC:
                    fputs("Caught SIGILL: (coprocessor error)\n", stderr);
                    break;
                case ILL_BADSTK:
                    fputs("Caught SIGILL: (internal stack error)\n", stderr);
                    break;
                default:
                    fputs("Caught SIGILL: Illegal Instruction\n", stderr);
                    break;
            }
            break;
        case SIGTERM:
            fputs("Caught SIGTERM: a termination request was sent to the program\n", stderr);
            break;
        case SIGABRT:
            fputs("Caught SIGABRT: usually caused by an abort() or assert()\n", stderr);
            break;
        default:
            break;
    }
    slam::print_stack_trace();
    _Exit(1);
}

static uint8_t alternate_stack[SIGSTKSZ];

void _setup_signal_handler() {
    {
        stack_t ss = {};
        /* malloc is usually used here, I'm not 100% sure my static allocation
           is valid but it seems to work just fine. */
        ss.ss_sp = (void *) alternate_stack;
        ss.ss_size = SIGSTKSZ;
        ss.ss_flags = 0;
        if (sigaltstack(&ss, NULL) != 0) { err(1, "sigaltstack"); }
    }

    {
        struct sigaction sig_action = {};
        sig_action.sa_sigaction = _posix_signal_handler;
        sigemptyset(&sig_action.sa_mask);
        sig_action.sa_flags = SA_SIGINFO | SA_ONSTACK;
        if (sigaction(SIGSEGV, &sig_action, NULL) != 0) { err(1, "sigaction"); }
        if (sigaction(SIGFPE, &sig_action, NULL) != 0) { err(1, "sigaction"); }
        if (sigaction(SIGINT, &sig_action, NULL) != 0) { err(1, "sigaction"); }
        if (sigaction(SIGILL, &sig_action, NULL) != 0) { err(1, "sigaction"); }
        if (sigaction(SIGTERM, &sig_action, NULL) != 0) { err(1, "sigaction"); }
        if (sigaction(SIGABRT, &sig_action, NULL) != 0) { err(1, "sigaction"); }
    }
}

#endif

namespace slam {

    static char const *G_global_program_name = "";

    /* -------------------------------------------------------------------------------------------------------------- */
    void print_stack_trace() {
#ifdef _IS_UNIX
        if (std::string(G_global_program_name) == "") {
            std::cerr << "[ERROR] The program name has not been set. `print_stack_trace()` will do nothing"
                      << std::endl;
        }
        static void *stack_traces[MAX_STACK_FRAMES];
        int i, trace_size = 0;
        char **messages = (char **) NULL;

        trace_size = backtrace(stack_traces, MAX_STACK_FRAMES);
        messages = backtrace_symbols(stack_traces, trace_size);

        for (i = 3; i < (trace_size - 1); ++i) {
            printf("  message for depth: [%d]: %s\n", i - 1, messages[i]);
//            if (_get_adress_line(G_global_program_name, stack_traces[i]) != 0) {
//                printf("  error determining line # for: %s\n", messages[i]);
//            }

        }
        if (messages) { free(messages); }
#endif
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void setup_signal_handler(int argc, char **argv) {
#ifdef _IS_UNIX
        G_global_program_name = argv[0];
        _setup_signal_handler();
#endif
    }


} // namespace slam
