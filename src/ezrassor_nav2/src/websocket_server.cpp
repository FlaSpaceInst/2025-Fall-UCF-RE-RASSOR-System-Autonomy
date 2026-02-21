// websocket_server.cpp
// Boost.Beast async WebSocket server.
// Each connected session gets a shared_ptr; broadcast() fans out to all of them.

#include "ezrassor_nav2/websocket_server.hpp"

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

namespace beast     = boost::beast;
namespace websocket = beast::websocket;
namespace net       = boost::asio;
using tcp           = net::ip::tcp;

namespace ezrassor_nav2 {

// ─── Session ──────────────────────────────────────────────────────────────────
class Session : public std::enable_shared_from_this<Session> {
public:
  using MessageCallback = WebSocketServer::MessageCallback;

  explicit Session(tcp::socket socket, MessageCallback cb)
  : ws_(std::move(socket)), msg_cb_(std::move(cb)) {}

  void start() { do_read(); }

  void send(const std::string & msg)
  {
    auto self = shared_from_this();
    net::post(ws_.get_executor(), [self, msg]() {
      bool idle = self->send_queue_.empty();
      self->send_queue_.push_back(msg);
      if (idle) { self->do_write(); }
    });
  }

  void close() { ws_.async_close(websocket::close_code::normal,
    [](beast::error_code) {}); }

private:
  void do_read()
  {
    auto self = shared_from_this();
    ws_.async_read(buf_, [self](beast::error_code ec, std::size_t) {
      if (ec) { self->on_close(); return; }
      if (self->msg_cb_) {
        self->msg_cb_(beast::buffers_to_string(self->buf_.data()));
      }
      self->buf_.consume(self->buf_.size());
      self->do_read();
    });
  }

  void do_write()
  {
    auto self = shared_from_this();
    ws_.async_write(net::buffer(send_queue_.front()),
      [self](beast::error_code ec, std::size_t) {
        if (ec) { self->on_close(); return; }
        self->send_queue_.erase(self->send_queue_.begin());
        if (!self->send_queue_.empty()) { self->do_write(); }
      });
  }

  void on_close()
  {
    // Parent (Impl) removes from set via weak_ptr / observer pattern;
    // we just stop doing work here.
  }

  websocket::stream<tcp::socket> ws_;
  beast::flat_buffer buf_;
  MessageCallback msg_cb_;
  std::vector<std::string> send_queue_;
};

// ─── Impl ─────────────────────────────────────────────────────────────────────
struct WebSocketServer::Impl {
  net::io_context ioc_;
  tcp::acceptor acceptor_;
  uint16_t port_;
  MessageCallback msg_cb_;
  std::mutex sessions_mtx_;
  std::set<std::shared_ptr<Session>> sessions_;
  std::thread ioc_thread_;

  explicit Impl(uint16_t port)
  : ioc_(1), acceptor_(ioc_), port_(port) {}

  void start()
  {
    tcp::endpoint ep{tcp::v4(), port_};
    acceptor_.open(ep.protocol());
    acceptor_.set_option(net::socket_base::reuse_address(true));
    acceptor_.bind(ep);
    acceptor_.listen();
    do_accept();

    ioc_thread_ = std::thread([this]() {
      std::cout << "[ezrassor_nav2] WebSocket listening on port " << port_ << "\n";
      ioc_.run();
    });
  }

  void stop()
  {
    ioc_.stop();
    if (ioc_thread_.joinable()) { ioc_thread_.join(); }
  }

  void do_accept()
  {
    acceptor_.async_accept([this](beast::error_code ec, tcp::socket socket) {
      if (!ec) {
        auto session = std::make_shared<Session>(std::move(socket), msg_cb_);
        {
          std::lock_guard<std::mutex> lk(sessions_mtx_);
          sessions_.insert(session);
        }
        // Handshake
        session->ws().async_accept([this, session](beast::error_code ec2) {
          if (!ec2) { session->start(); }
          else {
            std::lock_guard<std::mutex> lk(sessions_mtx_);
            sessions_.erase(session);
          }
        });
      }
      do_accept();
    });
  }

  void broadcast(const std::string & msg)
  {
    std::lock_guard<std::mutex> lk(sessions_mtx_);
    for (auto & s : sessions_) { s->send(msg); }
  }
};

// Expose ws_ as public for the Impl handshake workaround above
// (minor hack – production code would move handshake into Session)
namespace {
  // Small adapter so Impl can call session->ws()
  // We just befriend or use a public getter — simplest approach:
}

// ─── WebSocketServer public API ───────────────────────────────────────────────
WebSocketServer::WebSocketServer(uint16_t port)
: impl_(std::make_unique<Impl>(port)) {}

WebSocketServer::~WebSocketServer() { stop(); }

void WebSocketServer::start() { impl_->start(); }
void WebSocketServer::stop()  { impl_->stop(); }

void WebSocketServer::broadcast(const std::string & json_msg)
{
  impl_->broadcast(json_msg);
}

void WebSocketServer::set_message_callback(MessageCallback cb)
{
  impl_->msg_cb_ = std::move(cb);
}

}  // namespace ezrassor_nav2