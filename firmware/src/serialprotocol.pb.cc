// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: serialprotocol.proto

#include "serialprotocol.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG

namespace _pb = ::PROTOBUF_NAMESPACE_ID;
namespace _pbi = _pb::internal;

namespace inverted_pendulum {
PROTOBUF_CONSTEXPR RequestState::RequestState(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_._has_bits_)*/{}
  , /*decltype(_impl_._cached_size_)*/{}
  , /*decltype(_impl_.message_)*/{&::_pbi::fixed_address_empty_string, ::_pbi::ConstantInitialized{}}} {}
struct RequestStateDefaultTypeInternal {
  PROTOBUF_CONSTEXPR RequestStateDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~RequestStateDefaultTypeInternal() {}
  union {
    RequestState _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 RequestStateDefaultTypeInternal _RequestState_default_instance_;
PROTOBUF_CONSTEXPR ResponseState::ResponseState(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_._has_bits_)*/{}
  , /*decltype(_impl_._cached_size_)*/{}
  , /*decltype(_impl_.angle_)*/0} {}
struct ResponseStateDefaultTypeInternal {
  PROTOBUF_CONSTEXPR ResponseStateDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~ResponseStateDefaultTypeInternal() {}
  union {
    ResponseState _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 ResponseStateDefaultTypeInternal _ResponseState_default_instance_;
}  // namespace inverted_pendulum
static ::_pb::Metadata file_level_metadata_serialprotocol_2eproto[2];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_serialprotocol_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_serialprotocol_2eproto = nullptr;

const uint32_t TableStruct_serialprotocol_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::inverted_pendulum::RequestState, _impl_._has_bits_),
  PROTOBUF_FIELD_OFFSET(::inverted_pendulum::RequestState, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::inverted_pendulum::RequestState, _impl_.message_),
  0,
  PROTOBUF_FIELD_OFFSET(::inverted_pendulum::ResponseState, _impl_._has_bits_),
  PROTOBUF_FIELD_OFFSET(::inverted_pendulum::ResponseState, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::inverted_pendulum::ResponseState, _impl_.angle_),
  0,
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, -1, sizeof(::inverted_pendulum::RequestState)},
  { 8, 15, -1, sizeof(::inverted_pendulum::ResponseState)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::inverted_pendulum::_RequestState_default_instance_._instance,
  &::inverted_pendulum::_ResponseState_default_instance_._instance,
};

const char descriptor_table_protodef_serialprotocol_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\024serialprotocol.proto\022\021inverted_pendulu"
  "m\"\037\n\014RequestState\022\017\n\007message\030\001 \002(\t\"\036\n\rRe"
  "sponseState\022\r\n\005angle\030\001 \002(\001"
  ;
static ::_pbi::once_flag descriptor_table_serialprotocol_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_serialprotocol_2eproto = {
    false, false, 106, descriptor_table_protodef_serialprotocol_2eproto,
    "serialprotocol.proto",
    &descriptor_table_serialprotocol_2eproto_once, nullptr, 0, 2,
    schemas, file_default_instances, TableStruct_serialprotocol_2eproto::offsets,
    file_level_metadata_serialprotocol_2eproto, file_level_enum_descriptors_serialprotocol_2eproto,
    file_level_service_descriptors_serialprotocol_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_serialprotocol_2eproto_getter() {
  return &descriptor_table_serialprotocol_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_serialprotocol_2eproto(&descriptor_table_serialprotocol_2eproto);
namespace inverted_pendulum {

// ===================================================================

class RequestState::_Internal {
 public:
  using HasBits = decltype(std::declval<RequestState>()._impl_._has_bits_);
  static void set_has_message(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static bool MissingRequiredFields(const HasBits& has_bits) {
    return ((has_bits[0] & 0x00000001) ^ 0x00000001) != 0;
  }
};

RequestState::RequestState(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:inverted_pendulum.RequestState)
}
RequestState::RequestState(const RequestState& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  RequestState* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_._has_bits_){from._impl_._has_bits_}
    , /*decltype(_impl_._cached_size_)*/{}
    , decltype(_impl_.message_){}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  _impl_.message_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    _impl_.message_.Set("", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (from._internal_has_message()) {
    _this->_impl_.message_.Set(from._internal_message(), 
      _this->GetArenaForAllocation());
  }
  // @@protoc_insertion_point(copy_constructor:inverted_pendulum.RequestState)
}

inline void RequestState::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_._has_bits_){}
    , /*decltype(_impl_._cached_size_)*/{}
    , decltype(_impl_.message_){}
  };
  _impl_.message_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    _impl_.message_.Set("", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
}

RequestState::~RequestState() {
  // @@protoc_insertion_point(destructor:inverted_pendulum.RequestState)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void RequestState::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.message_.Destroy();
}

void RequestState::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void RequestState::Clear() {
// @@protoc_insertion_point(message_clear_start:inverted_pendulum.RequestState)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    _impl_.message_.ClearNonDefaultToEmpty();
  }
  _impl_._has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* RequestState::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // required string message = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          auto str = _internal_mutable_message();
          ptr = ::_pbi::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(ptr);
          #ifndef NDEBUG
          ::_pbi::VerifyUTF8(str, "inverted_pendulum.RequestState.message");
          #endif  // !NDEBUG
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  _impl_._has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* RequestState::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:inverted_pendulum.RequestState)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  // required string message = 1;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_message().data(), static_cast<int>(this->_internal_message().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "inverted_pendulum.RequestState.message");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_message(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:inverted_pendulum.RequestState)
  return target;
}

size_t RequestState::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:inverted_pendulum.RequestState)
  size_t total_size = 0;

  // required string message = 1;
  if (_internal_has_message()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_message());
  }
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData RequestState::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    RequestState::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*RequestState::GetClassData() const { return &_class_data_; }


void RequestState::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<RequestState*>(&to_msg);
  auto& from = static_cast<const RequestState&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:inverted_pendulum.RequestState)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_message()) {
    _this->_internal_set_message(from._internal_message());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void RequestState::CopyFrom(const RequestState& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:inverted_pendulum.RequestState)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RequestState::IsInitialized() const {
  if (_Internal::MissingRequiredFields(_impl_._has_bits_)) return false;
  return true;
}

void RequestState::InternalSwap(RequestState* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_impl_._has_bits_[0], other->_impl_._has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &_impl_.message_, lhs_arena,
      &other->_impl_.message_, rhs_arena
  );
}

::PROTOBUF_NAMESPACE_ID::Metadata RequestState::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_serialprotocol_2eproto_getter, &descriptor_table_serialprotocol_2eproto_once,
      file_level_metadata_serialprotocol_2eproto[0]);
}

// ===================================================================

class ResponseState::_Internal {
 public:
  using HasBits = decltype(std::declval<ResponseState>()._impl_._has_bits_);
  static void set_has_angle(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static bool MissingRequiredFields(const HasBits& has_bits) {
    return ((has_bits[0] & 0x00000001) ^ 0x00000001) != 0;
  }
};

ResponseState::ResponseState(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:inverted_pendulum.ResponseState)
}
ResponseState::ResponseState(const ResponseState& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  ResponseState* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_._has_bits_){from._impl_._has_bits_}
    , /*decltype(_impl_._cached_size_)*/{}
    , decltype(_impl_.angle_){}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  _this->_impl_.angle_ = from._impl_.angle_;
  // @@protoc_insertion_point(copy_constructor:inverted_pendulum.ResponseState)
}

inline void ResponseState::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_._has_bits_){}
    , /*decltype(_impl_._cached_size_)*/{}
    , decltype(_impl_.angle_){0}
  };
}

ResponseState::~ResponseState() {
  // @@protoc_insertion_point(destructor:inverted_pendulum.ResponseState)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void ResponseState::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void ResponseState::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void ResponseState::Clear() {
// @@protoc_insertion_point(message_clear_start:inverted_pendulum.ResponseState)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.angle_ = 0;
  _impl_._has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ResponseState::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // required double angle = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 9)) {
          _Internal::set_has_angle(&has_bits);
          _impl_.angle_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  _impl_._has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* ResponseState::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:inverted_pendulum.ResponseState)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  // required double angle = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteDoubleToArray(1, this->_internal_angle(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:inverted_pendulum.ResponseState)
  return target;
}

size_t ResponseState::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:inverted_pendulum.ResponseState)
  size_t total_size = 0;

  // required double angle = 1;
  if (_internal_has_angle()) {
    total_size += 1 + 8;
  }
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData ResponseState::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    ResponseState::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*ResponseState::GetClassData() const { return &_class_data_; }


void ResponseState::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<ResponseState*>(&to_msg);
  auto& from = static_cast<const ResponseState&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:inverted_pendulum.ResponseState)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_angle()) {
    _this->_internal_set_angle(from._internal_angle());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void ResponseState::CopyFrom(const ResponseState& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:inverted_pendulum.ResponseState)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ResponseState::IsInitialized() const {
  if (_Internal::MissingRequiredFields(_impl_._has_bits_)) return false;
  return true;
}

void ResponseState::InternalSwap(ResponseState* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_impl_._has_bits_[0], other->_impl_._has_bits_[0]);
  swap(_impl_.angle_, other->_impl_.angle_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ResponseState::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_serialprotocol_2eproto_getter, &descriptor_table_serialprotocol_2eproto_once,
      file_level_metadata_serialprotocol_2eproto[1]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace inverted_pendulum
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::inverted_pendulum::RequestState*
Arena::CreateMaybeMessage< ::inverted_pendulum::RequestState >(Arena* arena) {
  return Arena::CreateMessageInternal< ::inverted_pendulum::RequestState >(arena);
}
template<> PROTOBUF_NOINLINE ::inverted_pendulum::ResponseState*
Arena::CreateMaybeMessage< ::inverted_pendulum::ResponseState >(Arena* arena) {
  return Arena::CreateMessageInternal< ::inverted_pendulum::ResponseState >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
