// Host-only regressions using real EIM replies; no ESPHome runtime or radio access.
#include "protocol.h"
#include "check.h"
#include <cstdio>
#include <cstring>
using namespace esphome::redlink;

// Historical installation map is a fixture only; production learns its own permutation.
static constexpr std::array<uint8_t, 50> CAPTURE_CHANNELS{
    10, 70,  50, 111, 26, 87, 58, 119, 89, 28, 91, 30, 77,  16, 14, 75, 66, 6,  109, 48, 73, 12, 113, 52, 62,
    4,  117, 56, 115, 54, 32, 93, 83,  22, 97, 44, 60, 121, 99, 46, 34, 95, 18, 79,  81, 20, 24, 85,  8,  68};

static void discovery_and_persistence() {
  HopLearner learner;
  HopMapRecord record{};
  CHECK(!learner.ready() && !learner.save(record, 0x389e, 904195310));
  CHECK(learner.observe(0, 49, 0xfffff0));
  CHECK(!learner.observe(0, 49, 0xfffff0));  // Duplicate cannot confirm.
  CHECK(!learner.observe(0, 49, 0xffffe0));  // Old replay cannot confirm.
  CHECK(learner.confirmed() == 0);
  CHECK(learner.observe(0, 49, 0x10));  // Forward counter wrap.
  CHECK(learner.confirmed() == 1);
  for (uint8_t slot = 1; slot < HOP_COUNT; ++slot) {
    CHECK(learner.observe(slot, 49 - slot, 1000 + slot));
    CHECK(learner.observe(slot, 49 - slot, 2000 + slot));
  }
  CHECK(learner.ready() && learner.active_rank(0) == 49);
  CHECK(learner.save(record, 0x389e, 904195310));
  HopLearner loaded;
  CHECK(!loaded.load(record, 0x31fd, 904195310));
  CHECK(!loaded.load(record, 0x389e, 904195311));
  CHECK(loaded.load(record, 0x389e, 904195310));
  CHECK(loaded.cached() && !loaded.ready());
  CHECK(loaded.observe(4, 45, 3000));
  CHECK(!loaded.observe(4, 45, 3000));
  CHECK(!loaded.ready());
  CHECK(loaded.observe(5, 44, 3242) && loaded.ready());
  HopMapRecord corrupt = record;
  corrupt.bytes[12] ^= 1;
  HopLearner rejected;
  CHECK(!rejected.load(corrupt, 0x389e, 904195310) && !rejected.ready());
  corrupt = record;
  corrupt.bytes[11] = corrupt.bytes[10];  // Invalid permutation even with a correct CRC.
  const uint16_t checksum = crc16(corrupt.bytes.data(), 60);
  corrupt.bytes[60] = checksum >> 8;
  corrupt.bytes[61] = checksum;
  CHECK(!rejected.load(corrupt, 0x389e, 904195310));
  learner.reset(true);
  CHECK(learner.active_rank(0) == 49 && !learner.ready());
  CHECK(learner.observe(0, 1, 4000));
  HopMapRecord preserved = record;
  CHECK(!learner.save(preserved, 0x389e, 904195310) && preserved.bytes == record.bytes);
  CHECK(!learner.observe(1, 1, 4242) && learner.conflicted());  // Duplicate physical assignment.
  learner.reset();
  CHECK(learner.observe(0, 1, 5000));
  CHECK(!learner.observe(0, 2, 5242) && learner.conflicted());  // Conflicting slot.
  CHECK(!loaded.observe(5, 43, 6000) && loaded.conflicted() && !loaded.ready());
  HopLearner counter_conflict;
  CHECK(counter_conflict.observe(0, 0, 1000));
  CHECK(!counter_conflict.observe(1, 1, 1000) && counter_conflict.conflicted());
  CHECK(counter_conflict.known() == 1 && counter_conflict.confirmed() == 0);
  HopLearner epoch;
  CHECK(epoch.observe(0, 0, 1000));
  epoch.restart_clock_epoch();
  CHECK(epoch.observe(0, 0, 1000) && epoch.confirmed() == 0);
  CHECK(!epoch.observe(0, 0, 1000) && epoch.confirmed() == 0);
  CHECK(epoch.observe(0, 0, 1242) && epoch.confirmed() == 1);
  epoch.reset();
  CHECK(epoch.observe(0, 0, 1000, 1000));
  CHECK(epoch.observe(0, 0, 1000, 2101000) && epoch.confirmed() == 0);
  CHECK(!epoch.observe(0, 0, 1000, 2101001) && epoch.confirmed() == 0);
  CHECK(epoch.observe(0, 0, 1242, 2101059) && epoch.confirmed() == 1);
  HopLearner retained;
  CHECK(retained.load(record, 0x389e, 904195310));
  CHECK(retained.observe(0, 49, 1000));
  CHECK(retained.observe(1, 48, 1242) && retained.ready());
  retained.restart_clock_epoch();
  CHECK(!retained.ready() && retained.active_rank(0) == 49);
  CHECK(retained.observe(0, 49, 100) && !retained.ready());
  CHECK(retained.observe(1, 48, 342) && retained.ready());
  for (uint8_t rank = 0; rank < HOP_COUNT; ++rank)
    CHECK(candidate_channel(rank).sync == 0x636b + rank);
}

static Frame from_hex(const char *hex) {
  Frame f;
  const size_t length = std::strlen(hex);
  CHECK(length % 2 == 0 && length / 2 <= f.data.size());
  f.size = length / 2;
  for (size_t i = 0; i < f.size; i++) {
    unsigned byte;
    CHECK(std::sscanf(hex + 2 * i, "%2x", &byte) == 1);
    f.data[i] = byte;
  }
  return f;
}
static void crc(Frame &f) {
  const uint16_t checksum = crc16(f.data.data(), f.size - 2);
  f.data[f.size - 2] = checksum >> 8;
  f.data[f.size - 1] = checksum;
}
static const char *TEMPORARY = "3428ffffd6e0389e0000f01e01ff005a8046464a2e005cdc284e3c63464afc02802f8446c8a901070020"
                               "f00922c00045c8a001000fb4a5";
static const char *PERMANENT = "3428ffffd6e0389e0000f01e01ff005a8046464a56005cdc284e3c63464afc02802f8446c8a901070020"
                               "f00922c0004500a001000f0646";
static const char *PENDING = "3428ffffd6e0389e0000f01e81ff009a8080464b2fff5cdc80808080464afc02802f8480000000070000"
                             "f00922c0004500a001000f3cd0";
// User-labeled Emergency Heat, telemetry-mode-emergency68-20261107T014510Z.
static const char *EMERGENCY = "3428ffffc27a389e0000f01e01ff005a8046444a2e005cdc284e3c63464af802802f844600a901070020"
                               "f00922c0004500a001000fd962";

static const char *DISCOVERY = "152affffb14e389e0000f08a51b1c09eb1150bb1c0675ccc";
static const char *REGISTRATION = "1428ffffb14e389e0000f08944389ef0b14e01008076f6";
static const char *REGISTRY = "1228ffffb14e389e0000f0071250004300b14e2fbf";
static const char *ACK = "0d22ffffb14e389e0000f08252fa9582";

static bool same_frame(const Frame &a, const Frame &b) {
  return a.size == b.size && std::memcmp(a.data.data(), b.data.data(), a.size) == 0;
}

static void exchange(GatewayEnrollment &pairing, const Frame *reply = nullptr, bool transport_ok = true) {
  CHECK(valid_frame(pairing.outgoing()));
  pairing.transmitted();
  pairing.transport_complete(transport_ok);
  if (reply != nullptr)
    pairing.receive(*reply);
  pairing.finish_window();
}

static void fresh_identity_tests() {
  // Readdress captured reply shapes; do not depend on the historical B14E identity.
  for (const uint16_t id :
       {uint16_t(0x0001), uint16_t(0x00ff), uint16_t(0x0100), uint16_t(0xb14f), uint16_t(0xc27a), uint16_t(0xfffd)}) {
    Frame discovery = from_hex(DISCOVERY), registration = from_hex(REGISTRATION);
    Frame registry = from_hex(REGISTRY), ack = from_hex(ACK);
    for (Frame *reply : {&discovery, &registration, &registry, &ack}) {
      reply->data[4] = id >> 8;
      reply->data[5] = id;
      crc(*reply);
    }
    registration.data[16] = id >> 8;
    registration.data[17] = id;
    registry.data[17] = id >> 8;
    registry.data[18] = id;
    crc(registration);
    crc(registry);
    GatewayEnrollment pairing;
    pairing.start(0x389e, id);
    exchange(pairing, &discovery);
    exchange(pairing, &ack);
    const Frame registration_write = pairing.outgoing();
    CHECK(registration_write.data[6] == id >> 8 && registration_write.data[7] == (id & 0xff));
    CHECK(registration_write.data[15] == id >> 8 && registration_write.data[16] == (id & 0xff));
    exchange(pairing, &registration);
    const Frame commit = pairing.outgoing();
    CHECK(commit.data[17] == id >> 8 && commit.data[18] == (id & 0xff));
    exchange(pairing, &ack);
    Frame foreign = registry;
    foreign.data[18] ^= 1;  // Addressed to us but naming another gateway.
    crc(foreign);
    exchange(pairing, &foreign);
    CHECK(!pairing.verified());
    exchange(pairing, &registry);
    CHECK(pairing.verified());
  }
}

static void enrollment_tests() {
  using State = GatewayEnrollment::State;
  CHECK(same_frame(gateway_discovery(0xd6e0), from_hex("0c33fffffffed6e0f0000081115d1e")));
  CHECK(same_frame(gateway_discovery(0xb14e), from_hex("0c33fffffffeb14ef0000081116ccb")));
  CHECK(same_frame(gateway_announcement(0xb14e), from_hex("0c20fffffffeb14ef0000081602a94")));
  CHECK(same_frame(gateway_registration(0xd6e0), from_hex("1230fffffffed6e0f0000087820300d6e0f0d21900")));
  CHECK(same_frame(gateway_registration(0xb14e), from_hex("1230fffffffeb14ef0000087820300b14ef0d22162")));
  CHECK(same_frame(gateway_commit(0x389e, 0xb14e), from_hex("1200ffff389eb14ef00000071250004300b14ebd82")));
  CHECK(same_frame(gateway_registry_query(0x389e, 0xb14e), from_hex("0f10ffff389eb14ef0000004125000000829")));

  const Frame discovery = from_hex(DISCOVERY);
  const Frame registration = from_hex(REGISTRATION);
  const Frame registry = from_hex(REGISTRY);
  const Frame ack = from_hex(ACK);
  CHECK(discovery_reply(discovery, 0x389e, 0xb14e));
  CHECK(registration_reply(registration, 0x389e, 0xb14e));
  CHECK(registry_reply(registry, 0x389e, 0xb14e));
  CHECK(acknowledgement(ack, 0x389e, 0xb14e));
  CHECK(!registry_reply(ack, 0x389e, 0xb14e));
  CHECK(!registration_reply(ack, 0x389e, 0xb14e));
  CHECK(!discovery_reply(discovery, 0x389e, 0xb14f));
  CHECK(!registration_reply(registration, 0x389f, 0xb14e));
  CHECK(!registry_reply(registry, 0x389e, 0xb14f));

  // The observed discovery variant is type 2e, not an ordinary status response.
  Frame variant = discovery;
  variant.data[1] = 0x2e;
  crc(variant);
  CHECK(discovery_reply(variant, 0x389e, 0xb14e));
  Status status;
  CHECK(!collect_status(variant, status));
  uint8_t slot = 99;
  ClockListenWindow next;
  uint32_t phase = 0;
  CHECK(!clock_slot(variant, 0x389e, slot, phase, next));  // Discovery is not a broadcast clock.
  variant.data[1] = 0x28;
  crc(variant);
  CHECK(!discovery_reply(variant, 0x389e, 0xb14e));

  // CRC-valid foreign PAN, destination, source, flags, and endpoints never advance pairing.
  for (size_t i = 2; i <= 10; ++i) {
    Frame bad = discovery;
    bad.data[i] ^= 1;
    crc(bad);
    CHECK(!discovery_reply(bad, 0x389e, 0xb14e));
    bad = registration;
    bad.data[i] ^= 1;
    crc(bad);
    CHECK(!registration_reply(bad, 0x389e, 0xb14e));
    bad = registry;
    bad.data[i] ^= 1;
    crc(bad);
    CHECK(!registry_reply(bad, 0x389e, 0xb14e));
    bad = ack;
    bad.data[i] ^= 1;
    crc(bad);
    CHECK(!acknowledgement(bad, 0x389e, 0xb14e));
  }
  for (size_t i = 11; i < registration.size - 2; ++i) {
    Frame bad = registration;
    bad.data[i] ^= 1;
    crc(bad);
    CHECK(!registration_reply(bad, 0x389e, 0xb14e));
  }
  for (size_t i = 11; i < registry.size - 2; ++i) {
    Frame bad = registry;
    bad.data[i] ^= 1;
    crc(bad);
    CHECK(!registry_reply(bad, 0x389e, 0xb14e));
  }
  Frame bad = ack;
  bad.data[12] = 0x51;
  crc(bad);
  CHECK(!acknowledgement(bad, 0x389e, 0xb14e));
  bad = registration;
  bad.data[bad.size - 1] ^= 1;
  CHECK(!registration_reply(bad, 0x389e, 0xb14e));
  bad = discovery;
  --bad.size;
  bad.data[0] = bad.size - 3;
  crc(bad);
  CHECK(!discovery_reply(bad, 0x389e, 0xb14e));
  bad = registration;
  --bad.size;
  bad.data[0] = bad.size - 3;
  crc(bad);
  CHECK(!registration_reply(bad, 0x389e, 0xb14e));

  GatewayEnrollment pairing;
  CHECK(pairing.state() == State::IDLE && !pairing.verified() && pairing.outgoing().size == 0);
  pairing.start(0x389e, 0xb14e);
  pairing.receive(discovery);  // A delayed unsolicited response before transport is not proof.
  exchange(pairing, &ack);
  CHECK(pairing.state() == State::DISCOVER && !pairing.may_have_changed());
  exchange(pairing, &discovery);
  CHECK(pairing.state() == State::ANNOUNCE);
  exchange(pairing);  // Exactly one successful announcement; ACK is intentionally unnecessary.
  CHECK(pairing.state() == State::REGISTER && !pairing.may_have_changed());
  exchange(pairing, &ack);
  CHECK(pairing.state() == State::REGISTER && pairing.may_have_changed() && !pairing.verified());
  bad = registration;
  bad.data[16] = 0xd6;
  bad.data[17] = 0xe0;  // Correct addressing but wrong registered identity.
  crc(bad);
  exchange(pairing, &bad);
  CHECK(pairing.state() == State::REGISTER);
  exchange(pairing, &registration);
  CHECK(pairing.state() == State::COMMIT && !pairing.verified());
  CHECK(same_frame(pairing.outgoing(), gateway_commit(0x389e, 0xb14e)));
  exchange(pairing);  // TX completion without an ACK must not imply delivery.
  CHECK(pairing.state() == State::COMMIT && !pairing.verified());
  exchange(pairing, &ack);
  CHECK(pairing.state() == State::VERIFY && !pairing.verified());
  bad = registry;
  bad.data[17] = 0xd6;
  bad.data[18] = 0xe0;  // Registry still contains the old gateway.
  crc(bad);
  exchange(pairing, &bad);
  CHECK(pairing.state() == State::VERIFY && !pairing.verified());
  CHECK(same_frame(pairing.outgoing(), gateway_registry_query(0x389e, 0xb14e)));
  exchange(pairing, &ack);
  CHECK(pairing.state() == State::VERIFY && !pairing.verified());
  exchange(pairing, &registry);
  CHECK(pairing.verified() && pairing.state() == State::COMPLETE && pairing.outgoing().size == 0);

  pairing.start(0x389e, 0xb14e);
  for (unsigned i = 0; i < 5; ++i)
    exchange(pairing);
  CHECK(pairing.state() == State::FAILED && !pairing.may_have_changed());
  pairing.start(0x389e, 0xb14e);
  exchange(pairing, &discovery);
  exchange(pairing, nullptr, false);  // Underflow/timeout cannot advance the announcement.
  CHECK(pairing.state() == State::FAILED && !pairing.may_have_changed());
  pairing.start(0x389e, 0xb14e);
  exchange(pairing, &discovery);
  exchange(pairing);
  exchange(pairing, &registration, false);
  CHECK(pairing.state() == State::FAILED && pairing.may_have_changed() && !pairing.verified());
  pairing.start(0x389e, 0xb14e);
  exchange(pairing, &discovery);
  exchange(pairing);
  exchange(pairing, &registration);
  pairing.fail();  // Disable/cancel before commit: no additional outgoing frame.
  CHECK(pairing.outgoing().size == 0 && pairing.may_have_changed() && !pairing.verified());
  pairing.start(0x389e, 0xb14e);
  exchange(pairing, &discovery);
  exchange(pairing);
  exchange(pairing, &registration);
  exchange(pairing, &ack);
  for (unsigned i = 0; i < 5; ++i)
    exchange(pairing, &bad);
  CHECK(pairing.state() == State::FAILED && pairing.may_have_changed() && !pairing.verified());

  // ESPHome's distinct b14f identity appears in both the envelope and registration/registry payload.
  Frame next_discovery = discovery;
  next_discovery.data[5] = 0x4f;
  crc(next_discovery);
  Frame next_registration = registration;
  next_registration.data[5] = next_registration.data[17] = 0x4f;
  crc(next_registration);
  Frame next_registry = registry;
  next_registry.data[5] = next_registry.data[18] = 0x4f;
  crc(next_registry);
  pairing.start(0x389e, 0xb14f);
  CHECK(pairing.outgoing().data[7] == 0x4f);
  exchange(pairing, &discovery);
  CHECK(pairing.state() == State::DISCOVER);
  exchange(pairing, &next_discovery);
  exchange(pairing);
  CHECK(pairing.outgoing().data[16] == 0x4f);
  exchange(pairing, &next_registration);
  CHECK(pairing.state() == State::COMMIT && pairing.outgoing().data[18] == 0x4f);
  for (unsigned i = 0; i < 5; ++i)
    exchange(pairing);  // Missing ACKs still end in a bounded registry read.
  exchange(pairing, &next_registry);
  CHECK(pairing.verified());

  pairing.start_commit(0x389e, 0xb14e);
  CHECK(same_frame(pairing.outgoing(), gateway_commit(0x389e, 0xb14e)));
  CHECK(!pairing.may_have_changed());
  exchange(pairing);
  CHECK(pairing.state() == State::COMMIT && pairing.may_have_changed());
  exchange(pairing, &ack);
  CHECK(pairing.state() == State::VERIFY && !pairing.verified());
  exchange(pairing, &registry);
  CHECK(pairing.verified());  // Completion needs no discovery or registration.
}
static void emergency_heat_tests() {
  const Frame captured = from_hex(EMERGENCY);
  CHECK(valid_frame(captured) && peer_reply(captured, 0x389e, 0xc27a));
  Status emergency;
  CHECK(collect_status(captured, emergency) && actual_status(emergency));
  CHECK(emergency.thermostat[7] == 68 && emergency.thermostat[19] == 0xf8);
  CHECK(matches(emergency, emergency));
  std::array<Frame, 2> frames;
  size_t count = 0;
  Status expected;
  // Every unrelated writable field retains all mode byte bits, not merely the displayed HEAT.
  for (unsigned field = 0; field < 5; ++field) {
    Control control;
    if (field == 0)
      control.heat = 69;
    if (field == 1)
      control.cool = 75;
    if (field == 2)
      control.hold = 2;
    if (field == 3) {
      control.hold = 1;
      control.until = 91;
    }
    if (field == 4)
      control.fan = 1;
    CHECK(prepare_controls(emergency, control, 0x389e, 0xc27a, frames, count, expected));
    CHECK(count == 1 && valid_frame(frames[0]));
    CHECK(expected.thermostat[19] == 0xf8);
    if (field == 4) {
      CHECK(frames[0].size == 23 && expected.thermostat == emergency.thermostat);
    } else {
      CHECK(frames[0].size == 44 && frames[0].data[11 + 19] == 0xf8);
    }
    // Independently modeled actual readback must match the preserved emergency mode.
    Status readback = emergency;
    readback.thermostat[7] = expected.thermostat[7];
    readback.thermostat[8] = expected.thermostat[8];
    readback.thermostat[9] = expected.thermostat[9];
    readback.thermostat[11] = expected.thermostat[11];
    readback.thermostat[12] = expected.thermostat[12];
    readback.fan[9] = expected.fan[9];
    CHECK(matches(readback, expected));
    readback.thermostat[19] = 0xf9;  // Ordinary heat is not Emergency Heat.
    CHECK(!matches(readback, expected));
  }
  Control no_op;
  no_op.heat = 68;
  CHECK(prepare_controls(emergency, no_op, 0x389e, 0xc27a, frames, count, expected));
  CHECK(count == 0 && matches(emergency, expected));
  no_op.mode = 0;
  CHECK(prepare_controls(emergency, no_op, 0x389e, 0xc27a, frames, count, expected));
  CHECK(count == 0 && matches(emergency, expected));
  for (int mode : {1, 2, 3, 4}) {
    Status other = emergency;
    other.thermostat[19] = 0xf8 | mode;
    CHECK(actual_status(other) && !matches(other, emergency) && !matches(emergency, other));
    Control leave;
    leave.mode = mode;
    CHECK(prepare_controls(emergency, leave, 0x389e, 0xc27a, frames, count, expected));
    CHECK(count == 1 && expected.thermostat[19] == (0xf8 | mode));
    CHECK(frames[0].data[11 + 19] == (0xf8 | mode));
    CHECK(matches(other, expected));
    Control enter;
    enter.mode = 0;
    CHECK(prepare_controls(other, enter, 0x389e, 0xc27a, frames, count, expected));
    CHECK(count == 1 && frames[0].size == 44 && valid_frame(frames[0]));
    CHECK(frames[0].data[11 + 19] == 0xf8 && expected.thermostat[19] == 0xf8);
    CHECK((frames[0].data[11 + 9] & 0x78) == (other.thermostat[9] & 0x78));
    for (size_t offset : {7, 8, 11, 12, 17, 18, 20, 22, 23})
      CHECK(frames[0].data[11 + offset] == other.thermostat[offset]);
    CHECK(matches(emergency, expected) && !matches(other, expected));
    // Only the low three mode bits change; upper bits must come from this pre-read.
    other.thermostat[19] = 0xa8 | mode;
    CHECK(prepare_controls(other, enter, 0x389e, 0xc27a, frames, count, expected));
    CHECK(count == 1 && frames[0].data[11 + 19] == 0xa8 && expected.thermostat[19] == 0xa8);
  }
  for (int mode : {5, 6, 7}) {
    Control unsupported;
    unsupported.mode = mode;
    CHECK(!prepare_controls(emergency, unsupported, 0x389e, 0xc27a, frames, count, expected));
    Status unknown = emergency;
    unknown.thermostat[19] = 0xf8 | mode;
    CHECK(!actual_status(unknown) && !matches(unknown, emergency) && !matches(emergency, unknown));
    CHECK(!prepare_controls(unknown, no_op, 0x389e, 0xc27a, frames, count, expected));
  }
}

int main() {
  // Original gateway ACK, decoded after the EIM's status reply in gateway-close-upper.cs8.
  CHECK(same_frame(status_reply_ack(0x389e, 0xd6e0), from_hex("0d22ffff389ed6e0f00000825228680b")));
  discovery_and_persistence();
  enrollment_tests();
  fresh_identity_tests();
  emergency_heat_tests();
  Frame reply = from_hex(TEMPORARY);
  CHECK(peer_reply(reply, 0x389e, 0xd6e0));
  CHECK(!peer_reply(reply, 0x389e, 0xd7e0));
  Status s;
  CHECK(collect_status(reply, s) && s.complete());
  CHECK(s.thermostat[7] == 70 && s.thermostat[8] == 74);
  CHECK((s.thermostat[19] & 7) == 4 && (s.fan[9] & 3) == 3);
  CHECK((s.thermostat[11] & 0x7f) == 92);
  Status pending;
  CHECK(collect_status(from_hex(PENDING), pending) && !pending.complete());
  CHECK(!actual_status(pending));
  Status accumulated = s;
  CHECK(collect_status(from_hex(PENDING), accumulated) && !accumulated.have_thermostat);
  Frame pending_fan = reply;
  pending_fan.data[47] = 0x85;
  crc(pending_fan);
  accumulated = s;
  CHECK(collect_status(pending_fan, accumulated) && !accumulated.have_fan);
  Status pending_flags = s;
  pending_flags.thermostat[4] = 0x9a;
  CHECK(!actual_status(pending_flags) && !matches(pending_flags, s));
  pending_flags = s;
  pending_flags.fan[4] = 0x85;
  CHECK(!actual_status(pending_flags) && !matches(pending_flags, s));

  Frame malformed = reply;
  malformed.data[42] = 0;  // Bad inter-record endpoint, with otherwise valid CRC.
  crc(malformed);
  Status atomic;
  CHECK(!collect_status(malformed, atomic) && !atomic.have_thermostat && !atomic.have_fan);
  malformed = reply;
  malformed.data[7] ^= 1;
  CHECK(!valid_frame(malformed));

  Status permanent;
  CHECK(collect_status(from_hex(PERMANENT), permanent));
  std::array<Frame, 2> frames;
  size_t count = 0;
  Status expected;
  Control c;
  c.heat = 69;
  CHECK(prepare_controls(permanent, c, 0x389e, 0xd6e0, frames, count, expected));
  CHECK(count == 1 && valid_frame(frames[0]));
  CHECK(expected.thermostat[7] == 69 && expected.thermostat[8] == 74);
  CHECK(((expected.thermostat[9] >> 3) & 3) == 2 && ((expected.thermostat[9] >> 5) & 3) == 2);
  CHECK(expected.thermostat[19] == permanent.thermostat[19]);

  c = Control{};
  c.fan = 1;
  CHECK(prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
  CHECK(count == 1 && frames[0].size == 23 && (expected.fan[9] & 3) == 1);
  CHECK(expected.thermostat == s.thermostat);

  // Repeated HA targets must not restart the thermostat's pending-write cycle.
  c = Control{};
  c.heat = 70;
  c.cool = 74;
  c.fan = 3;
  c.hold = 1;
  CHECK(prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
  CHECK(count == 0 && matches(s, expected));
  c.heat = 71;
  CHECK(prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
  CHECK(count == 1 && frames[0].size == 44 && expected.thermostat[7] == 71);
  c.heat = 70;
  c.fan = 1;
  CHECK(prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
  CHECK(count == 1 && frames[0].size == 23 && (expected.fan[9] & 3) == 1);

  // A newer partial adjustment supersedes that field without dropping others.
  Control latest;
  latest.heat = 70;
  latest.fan = 1;
  latest.hold = 1;
  latest.until = 91;
  Control newer;
  newer.heat = 71;
  latest.merge(newer);
  CHECK(prepare_controls(s, latest, 0x389e, 0xd6e0, frames, count, expected));
  CHECK(expected.thermostat[7] == 71 && (expected.fan[9] & 3) == 1);
  CHECK((expected.thermostat[11] & 0x7f) == 91);
  newer = Control{};
  newer.hold = 2;
  latest.merge(newer);
  CHECK(latest.until == -1);
  CHECK(prepare_controls(s, latest, 0x389e, 0xd6e0, frames, count, expected));
  CHECK(((expected.thermostat[9] >> 3) & 3) == 2);

  c = Control{};
  c.hold = 1;
  c.until = 91;  // 22:45, retaining the unrelated high flag bits.
  CHECK(prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
  CHECK((expected.thermostat[11] & 0x7f) == 91 && (expected.thermostat[12] & 0x7f) == 91);
  CHECK((expected.thermostat[12] & 0x80) == (s.thermostat[12] & 0x80));
  Status actual = expected;
  actual.thermostat[4] = 0x5a;  // Actual status record, not the write's 0x9a.
  actual.thermostat[6] ^= 1;    // Unrelated live measurement.
  CHECK(matches(actual, expected));
  actual.thermostat[11] = 90;
  CHECK(!matches(actual, expected));

  c = Control{};
  c.hold = 0;
  Status unsafe = s;
  unsafe.thermostat[17] = 54;
  CHECK(!prepare_controls(unsafe, c, 0x389e, 0xd6e0, frames, count, expected));
  c = Control{};
  c.heat = 81;
  CHECK(!prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
  c.heat = 55;
  c.cool = 57;
  CHECK(prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
  c.heat = 78;
  c.cool = 80;
  CHECK(prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
  c.cool = 79;
  CHECK(!prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
  for (int invalid : {54, 81}) {
    c = Control{};
    c.heat = invalid;
    CHECK(!prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
    c = Control{};
    c.cool = invalid;
    CHECK(!prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
  }
  c = Control{};
  c.heat = 55;
  c.cool = 80;
  CHECK(prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
  for (const auto pair : {std::array<int, 2>{55, 55}, {55, 56}, {79, 80}, {80, 80}, {80, 55}}) {
    c.heat = pair[0];
    c.cool = pair[1];
    CHECK(!prepare_controls(s, c, 0x389e, 0xd6e0, frames, count, expected));
  }
  c = Control{};
  c.hold = 0;
  Status expanded_schedule = s;
  expanded_schedule.thermostat[17] = 55;
  expanded_schedule.thermostat[18] = 80;
  CHECK(prepare_controls(expanded_schedule, c, 0x389e, 0xd6e0, frames, count, expected));
  CHECK(expected.thermostat[7] == 55 && expected.thermostat[8] == 80);
  expanded_schedule.thermostat[18] = 81;
  CHECK(!prepare_controls(expanded_schedule, c, 0x389e, 0xd6e0, frames, count, expected));

  // Recorded channel-10 clock: slot zero, including the k=3 counter offset.
  Frame clock = from_hex("1502ffff389e389e0000ff8a5145b69c45b3b445b1af2a50");
  uint8_t slot = 99;
  ClockListenWindow next;
  uint32_t phase = 0;
  CHECK(clock_slot(clock, 0x389e, slot, phase, next) && slot == 0 && phase == 1261);
  CHECK(next.slot == 48 && next.copy == 0);  // k=3: next burst starts at base 47 + 1.
  CHECK(!clock_slot(clock, 0x389f, slot, phase, next));
  // Unmodified 144-byte clock/status broadcast from the gateway Connect capture.
  // It must not be discarded merely because it spans the CC1101's 64-byte FIFO.
  Frame extended = from_hex("8d06ffff389e389e0000ff8a517e51fd7e50f97e50a1ff0d01f40049ff110a030a01201d1c"
                            "ff1401f0005000118020801d801c010a0003000a0001ff0401f90040ff080045004402330003"
                            "ff2401ea006020010a0616032101180111031c011d030101170609010501230112010301ff31"
                            "ff0a84040086032002390000ff0f313e804a0300d77a5128fe208000009c31");
  CHECK(clock_slot(extended, 0x389e, slot, phase, next) && slot == 9 && phase == 348);
  CHECK(next.slot == 10 && next.copy == 2 && next.sync_us == 59082);
  extended.data[extended.size - 1] ^= 1;
  CHECK(!clock_slot(extended, 0x389e, slot, phase, next));
  // Synthesize the same valid relationships across a 24-bit counter wrap.
  const uint32_t counters[] = {8, 0xfffff6, 0xfffff6};
  for (int i = 0; i < 3; i++) {
    clock.data[13 + 3 * i] = counters[i] >> 16;
    clock.data[14 + 3 * i] = counters[i] >> 8;
    clock.data[15 + 3 * i] = counters[i];
  }
  crc(clock);
  CHECK(clock_slot(clock, 0x389e, slot, phase, next) && slot == 0 && phase == 18 && next.slot == 1);
  clock.data[15]++;
  crc(clock);
  CHECK(!clock_slot(clock, 0x389e, slot, phase, next));

  // Actual successive broadcasts, including slots absent from the old 47-slot profile.
  // Receive tuning follows each copy and returns to the next burst's first channel.
  struct Capture {
    const char *hex;
    uint8_t channel, next_channel;
  };
  const Capture captures[] = {
      {"1502ffff389e389e0000ff8a517b7f0a7b7ef87b7ec10be7", 87, 58},
      {"1502ffff389e389e0000ff8a517b7ffc7b7ef87b7ec1f287", 58, 119},
      {"1502ffff389e389e0000ff8a517b80ee7b7ef87b7ec11387", 119, 89},
      {"1502ffff389e389e0000ff8a517b81e07b7ef87b7ec18d62", 89, 58},
      {"1f02ffff389e389e0000ff8a517c6fb57c6fa37c6f61ff084045004402510000c13d", 58, 119},
      {"1502ffff389e389e0000ff8a5159286f59276b5925b3163a", 95, 18},
      {"1502ffff389e389e0000ff8a51592a5359276b5925b309f6", 79, 95},
      {"1502ffff389e389e0000ff8a515a18285a18165a1653d103", 95, 18},
      {"1502ffff389e389e0000ff8a515a1a0c5a18165a1653d74f", 79, 81},
      {"1502ffff389e389e0000ff8a515a1afe5a18165a1653aa6c", 81, 18},
  };
  for (const auto &capture : captures) {
    CHECK(clock_slot(from_hex(capture.hex), 0x389e, slot, phase, next));
    CHECK(CAPTURE_CHANNELS[slot] == capture.channel);
    CHECK(CAPTURE_CHANNELS[next.slot] == capture.next_channel);
  }
  // Missing copies must advance reception, not strand it on an old frequency.
  // Include slot 49 -> 0, the final-copy -> next-burst transition, and micros wrap.
  ClockListenWindow missed{49, 1, 0xfffff000};
  missed.advance();
  CHECK(missed.slot == 0 && missed.copy == 2 && missed.sync_us == uint32_t(0xfffff000U + 59082U));
  missed.advance();
  CHECK(missed.slot == 1 && missed.copy == 3);
  missed.advance();
  CHECK(missed.slot == 49 && missed.copy == 0);
  CHECK(missed.sync_us == uint32_t(0xfffff000ULL + 2 * 59082 + 14864502));
  std::puts(
      "RedLINK protocol: enrollment, actual status, clock wrap, and captured multi-channel clock tracking passed");
}
