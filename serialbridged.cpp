#include <fmt/format.h>
#include <systemd/sd-daemon.h>

#include <phosphor-logging/log.hpp>

#include <sdbusplus/bus.hpp>
#include <sdbusplus/slot.hpp>
#include <sdbusplus/server/interface.hpp>
#include <sdbusplus/vtable.hpp>
#include <sdeventplus/event.hpp>
#include <sdeventplus/source/io.hpp>
#include <sdeventplus/source/signal.hpp>
#include <stdplus/exception.hpp>
#include <stdplus/fd/create.hpp>
#include <stdplus/fd/ops.hpp>
#include <stdplus/signal.hpp>

#include <CLI/CLI.hpp>

#define BM_START             0xA0
#define BM_STOP              0xA5
#define BM_HANDSHAKE         0xA6
#define BM_ESCAPE            0xAA

#define MSG_IDLE             0
#define MSG_IN_PROGRESS      1

#define IPMI_SERIAL_MIN_REQ  7

using namespace phosphor::logging;

using sdeventplus::source::IO;
using sdeventplus::source::Signal;
using stdplus::fd::OpenAccess;
using stdplus::fd::OpenFlag;
using stdplus::fd::OpenFlags;

using sdbusplus::bus::bus;
using sdbusplus::message::message;
using sdbusplus::slot::slot;

std::string device = "ttyAMA0";
bool verbose = 0;

std::vector<uint8_t> request;

uint8_t ctx_state = MSG_IDLE;
uint8_t ctx_escape = 0;

static constexpr uint8_t netFnShift = 2;
static constexpr uint8_t lunMask = (1 << netFnShift) - 1;

constexpr sdbusplus::vtable::vtable_t dbusMethods[] = {
    sdbusplus::vtable::start(), sdbusplus::vtable::end(),
};

/*
 *  Table of special characters
 */
static const struct {
  uint8_t character;
  uint8_t escape;
} characters[] = {
  { BM_START,   0xB0 }, /* start */
  { BM_STOP,    0xB5 }, /* stop */
  { BM_HANDSHAKE, 0xB6 }, /* packet handshake */
  { BM_ESCAPE,  0xBA }, /* data escape */
  { 0x1B, 0x3B }      /* escape */
};

/* ipmi_csum  -  calculate an ipmi checksum
 *
 * @d:    buffer to check
 * @s:    position in buffer to start checksum from
 */
uint8_t
ipmi_csum(uint8_t * d, int s)
{
  uint8_t c = 0;
  for (; s > 0; s--, d++)
    c += *d;
  return -c;
}

/*
 *  Return escaped character for the given one
 */
static inline uint8_t
serial_bm_get_escaped_char(uint8_t c)
{
  int i;

  for (i = 0; i < 5; i++) {
    if (characters[i].character == c) {
      return characters[i].escape;
    }
  }

  return c;
}

/*
 *  Return unescaped character for the given one
 */
static inline uint8_t
serial_bm_get_unescaped_char(uint8_t c)
{
  int i;

  for (i = 0; i < 5; i++) {
    if (characters[i].escape == c) {
      return characters[i].character;
    }
  }

  return c;
}

/*
 *  Parse IPMI Serial Request State Machine
 */
int parseIncomingRequest(std::span<uint8_t> &in, std::vector<uint8_t> &out) {

    for (auto c : in) {

        if (c == BM_START) // START
        {
            ctx_state = MSG_IN_PROGRESS;
            ctx_escape = 0;
        }
        else if (ctx_state != MSG_IN_PROGRESS)
        {
            continue;
        }
        else if (ctx_escape)
        {
            uint8_t tmp;
            tmp = serial_bm_get_unescaped_char(c);

            if (tmp == c)
            {
                // error, then reset
                ctx_state = MSG_IDLE;
                continue;
            }

            out.push_back(tmp);
            ctx_escape = 0;
        }
        else if (c == BM_ESCAPE)
        {
            ctx_escape = 1;
            continue;
        }
        else if (c == BM_STOP) // STOP
        {
            ctx_state = MSG_IDLE;
            return true;
        }
        else if (c == BM_HANDSHAKE)
        {
            // just skip it
            continue;
        }
        else if (ctx_state == MSG_IN_PROGRESS)
        {
            out.push_back(c);
        }
    }

    return 0;
}

/*
 *  Encapsluate response to avoid escape character
 */
uint8_t checkEscapeCharacter(std::vector<uint8_t>& in, uint8_t c)
{
    uint8_t escape = serial_bm_get_escaped_char(c);
    if (escape != c) {
        in.push_back(BM_ESCAPE);
    }

    in.push_back(escape);

    return c;
}

/*
 *  Write function
 */
void write(stdplus::Fd& uart, uint8_t rsAddr,
           uint8_t rqAddr, uint8_t seq, message&& m)
{
    std::vector<uint8_t> buffer;
    std::span<uint8_t> out;

    try
    {
        if (m.is_method_error())
        {
            // Extra copy to workaround lack of `const sd_bus_error` constructor
            auto error = *m.get_error();
            throw sdbusplus::exception::SdBusError(&error, "ipmid response");
        }

        std::tuple<uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>
            ret;
        m.read(ret);

        const auto& [netfn, lun, cmd, cc, data] = ret;
        uint8_t csum = 0;

        buffer.push_back(BM_START);

        csum += checkEscapeCharacter(buffer, rqAddr);
        csum += checkEscapeCharacter(buffer, (netfn << netFnShift) | (lun & lunMask));
        checkEscapeCharacter(buffer, -csum); // csum1

        csum = 0;

        csum += checkEscapeCharacter(buffer, rsAddr);
        csum += checkEscapeCharacter(buffer, (seq << netFnShift) | (lun & lunMask));
        csum += checkEscapeCharacter(buffer, cmd);
        csum += checkEscapeCharacter(buffer, cc);

        for (auto c : data) {
            csum += checkEscapeCharacter(buffer, c);
        }

        checkEscapeCharacter(buffer, -csum); // csum2

        buffer.push_back(BM_STOP);

        out = std::span<uint8_t>(buffer.begin(), buffer.end());

        if (verbose)
        {
            std::string msgToLog =
                "Write serial request message with"
                " len=" +
                std::to_string(buffer.size()) + " netfn=" + std::to_string(netfn) +
                " lun=" + std::to_string(lun) + " cmd=" + std::to_string(cmd) +
                " seq=" + std::to_string(seq);
            log<level::ERR>(msgToLog.c_str());

            std::string msgData = "Tx: ";
            for (auto c : buffer) {
                msgData += std::format("{:#x} ", c);
            }
            log<level::ERR>(msgData.c_str());
        }
    }
    catch (const std::exception& e)
    {
        fmt::print(stderr, "IPMI Response failure: {}\n", e.what());

        buffer.push_back(1 << 2);
        buffer.push_back(0);
        buffer.push_back(0xff);
        out = std::span<uint8_t>(buffer.begin(), buffer.end());
    }

    stdplus::fd::writeExact(uart, out);
}

/*
 *  Read function
 */
void read(stdplus::Fd& uart, bus& bus, slot& outstanding)
{
    std::array<uint8_t, 1024> buffer;
    auto in = stdplus::fd::read(uart, buffer);

    if (in.empty())
    {
        return;
    }

    if (outstanding)
    {
        fmt::print(stderr, "Canceling outstanding request \n");
        outstanding = slot(nullptr);
    }

    if (!parseIncomingRequest(in, request)) {
        fmt::print(stderr, "Wait for STOP byte ... \n");
        return;
    }

    if (request.size() < IPMI_SERIAL_MIN_REQ)
    {
        fmt::print(stderr, "Invalid request length, ignoring \n");
        request.clear();
        return;
    }

    if (ipmi_csum(&request[0], 3))
    {
        fmt::print(stderr, "Invalid request checksum 1 \n");
        request.clear();
        return;
    }

    if (ipmi_csum(&request[3], request.size() - 3))
    {
        fmt::print(stderr, "Invalid request checksum 2 \n");
        request.clear();
        return;
    }

    auto m = bus.new_method_call("xyz.openbmc_project.Ipmi.Host",
                                 "/xyz/openbmc_project/Ipmi",
                                 "xyz.openbmc_project.Ipmi.Server", "execute");

    std::map<std::string, std::variant<int>> options;
    uint8_t rsAddr = request[0];
    uint8_t netfn = request[1] >> netFnShift;
    uint8_t lun = request[1] & lunMask;
    uint8_t rqAddr = request[3];
    uint8_t seq = request[4] >> netFnShift;
    uint8_t cmd = request[5];

    std::span req_span{request.begin(), request.end() - 1}; // not include checksum 2
    m.append(netfn, lun, cmd, req_span.subspan(6), options);

    if (verbose)
    {
        std::string msgToLog =
            "Read serial request message with"
            " len=" +
            std::to_string(request.size()) + " netfn=" + std::to_string(netfn) +
            " lun=" + std::to_string(lun) + " cmd=" + std::to_string(cmd) +
            " seq=" + std::to_string(seq);
        log<level::ERR>(msgToLog.c_str());

        std::string msgData = "Rx: ";
        for (auto c : request) {
            msgData += std::format("{:#x} ", c);
        }
        log<level::ERR>(msgData.c_str());
    }

    outstanding = m.call_async(
        stdplus::exception::ignore(
            [&outstanding, &uart, rsAddrCap{rsAddr}, rqAddrCap{rqAddr}, seqCap{seq}]
              (message&& m) {
                  outstanding = slot(nullptr);
                  write(uart, rsAddrCap, rqAddrCap, seqCap, std::move(m));
              }));

    request.clear();

    return;
}

int main(int argc, char* argv[])
{
    // Parse input parameter
    CLI::App app("UART IPMI Bridge");

    app.add_option("-d,--device", device,
                   "select uart device, default is ttyAMA0");
    app.add_option("-v,--verbose", verbose, "enable debug message");

    CLI11_PARSE(app, argc, argv);

    // Set up DBus and event loop
    auto event = sdeventplus::Event::get_default();
    auto bus = sdbusplus::bus::new_default();
    bus.attach_event(event.get(), SD_EVENT_PRIORITY_NORMAL);

    // Configure basic signal handling
    auto exit_handler = [&event](Signal&, const struct signalfd_siginfo*) {
        fmt::print(stderr, "Interrupted, Exiting\n");
        event.exit(0);
    };
    stdplus::signal::block(SIGINT);
    Signal sig_init(event, SIGINT, exit_handler);
    stdplus::signal::block(SIGTERM);
    Signal sig_term(event, SIGTERM, exit_handler);

    // Open an FD for the UART channel
    stdplus::ManagedFd uart = stdplus::fd::open(
        std::format("/dev/{}", device),
        OpenFlags(OpenAccess::ReadWrite).set(OpenFlag::NonBlock));
    sdbusplus::slot_t slot(nullptr);

    // Add a reader to the bus for handling inbound IPMI
    IO ioSource(
        event, uart.get(), EPOLLIN | EPOLLET,
        stdplus::exception::ignore(
            [&uart, &bus, &slot](IO&, int, uint32_t) {
                read(uart, bus, slot);
            }));

    // Allow processes to affect the state machine
    std::string dbusChannel = "ipmi_serial";
    auto obj = "/xyz/openbmc_project/Ipmi/Channel/" + dbusChannel;
    auto srv = "xyz.openbmc_project.Ipmi.Channel." + dbusChannel;
    auto intf = sdbusplus::server::interface::interface(
                    bus, obj.c_str(), "xyz.openbmc_project.Ipmi.Channel.ipmi_serial",
                    dbusMethods, nullptr);

    bus.request_name(srv.c_str());

    sd_notify(0, "READY=1");

    return event.loop();
}
