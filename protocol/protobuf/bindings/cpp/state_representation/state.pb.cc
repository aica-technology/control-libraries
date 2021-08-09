// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: state_representation/state.proto

#include "state_representation/state.pb.h"

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
namespace state_representation {
namespace proto {
constexpr State::State(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : name_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , type_(0)

  , empty_(false)
  , timestamp_(int64_t{0}){}
struct StateDefaultTypeInternal {
  constexpr StateDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~StateDefaultTypeInternal() {}
  union {
    State _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT StateDefaultTypeInternal _State_default_instance_;
}  // namespace proto
}  // namespace state_representation
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_state_5frepresentation_2fstate_2eproto[1];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_state_5frepresentation_2fstate_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_state_5frepresentation_2fstate_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_state_5frepresentation_2fstate_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::state_representation::proto::State, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::state_representation::proto::State, name_),
  PROTOBUF_FIELD_OFFSET(::state_representation::proto::State, type_),
  PROTOBUF_FIELD_OFFSET(::state_representation::proto::State, empty_),
  PROTOBUF_FIELD_OFFSET(::state_representation::proto::State, timestamp_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::state_representation::proto::State)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::state_representation::proto::_State_default_instance_),
};

const char descriptor_table_protodef_state_5frepresentation_2fstate_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n state_representation/state.proto\022\032stat"
  "e_representation.proto\"l\n\005State\022\014\n\004name\030"
  "\001 \001(\t\0223\n\004type\030\002 \001(\0162%.state_representati"
  "on.proto.StateType\022\r\n\005empty\030\003 \001(\010\022\021\n\ttim"
  "estamp\030\004 \001(\003*\355\003\n\tStateType\022\t\n\005STATE\020\000\022\022\n"
  "\016CARTESIANSTATE\020\001\022\027\n\023DUALQUATERNIONSTATE"
  "\020\002\022\016\n\nJOINTSTATE\020\003\022\022\n\016JACOBIANMATRIX\020\004\022\016"
  "\n\nTRAJECTORY\020\005\022\022\n\016GEOMETRY_SHAPE\020\006\022\026\n\022GE"
  "OMETRY_ELLIPSOID\020\007\022\024\n\020PARAMETER_DOUBLE\020\010"
  "\022\032\n\026PARAMETER_DOUBLE_ARRAY\020\t\022\022\n\016PARAMETE"
  "R_BOOL\020\n\022\030\n\024PARAMETER_BOOL_ARRAY\020\013\022\024\n\020PA"
  "RAMETER_STRING\020\014\022\032\n\026PARAMETER_STRING_ARR"
  "AY\020\r\022\034\n\030PARAMETER_CARTESIANSTATE\020\016\022\033\n\027PA"
  "RAMETER_CARTESIANPOSE\020\017\022\030\n\024PARAMETER_JOI"
  "NTSTATE\020\020\022\034\n\030PARAMETER_JOINTPOSITIONS\020\021\022"
  "\027\n\023PARAMETER_ELLIPSOID\020\022\022\024\n\020PARAMETER_MA"
  "TRIX\020\023\022\024\n\020PARAMETER_VECTOR\020\024b\006proto3"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_state_5frepresentation_2fstate_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_state_5frepresentation_2fstate_2eproto = {
  false, false, 676, descriptor_table_protodef_state_5frepresentation_2fstate_2eproto, "state_representation/state.proto", 
  &descriptor_table_state_5frepresentation_2fstate_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_state_5frepresentation_2fstate_2eproto::offsets,
  file_level_metadata_state_5frepresentation_2fstate_2eproto, file_level_enum_descriptors_state_5frepresentation_2fstate_2eproto, file_level_service_descriptors_state_5frepresentation_2fstate_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_state_5frepresentation_2fstate_2eproto_getter() {
  return &descriptor_table_state_5frepresentation_2fstate_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_state_5frepresentation_2fstate_2eproto(&descriptor_table_state_5frepresentation_2fstate_2eproto);
namespace state_representation {
namespace proto {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* StateType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_state_5frepresentation_2fstate_2eproto);
  return file_level_enum_descriptors_state_5frepresentation_2fstate_2eproto[0];
}
bool StateType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
      return true;
    default:
      return false;
  }
}


// ===================================================================

class State::_Internal {
 public:
};

State::State(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:state_representation.proto.State)
}
State::State(const State& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (!from._internal_name().empty()) {
    name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_name(), 
      GetArenaForAllocation());
  }
  ::memcpy(&type_, &from.type_,
    static_cast<size_t>(reinterpret_cast<char*>(&timestamp_) -
    reinterpret_cast<char*>(&type_)) + sizeof(timestamp_));
  // @@protoc_insertion_point(copy_constructor:state_representation.proto.State)
}

void State::SharedCtor() {
name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&type_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&timestamp_) -
    reinterpret_cast<char*>(&type_)) + sizeof(timestamp_));
}

State::~State() {
  // @@protoc_insertion_point(destructor:state_representation.proto.State)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void State::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  name_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void State::ArenaDtor(void* object) {
  State* _this = reinterpret_cast< State* >(object);
  (void)_this;
}
void State::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void State::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void State::Clear() {
// @@protoc_insertion_point(message_clear_start:state_representation.proto.State)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  name_.ClearToEmpty();
  ::memset(&type_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&timestamp_) -
      reinterpret_cast<char*>(&type_)) + sizeof(timestamp_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* State::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // string name = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          auto str = _internal_mutable_name();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "state_representation.proto.State.name"));
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // .state_representation.proto.StateType type = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_type(static_cast<::state_representation::proto::StateType>(val));
        } else goto handle_unusual;
        continue;
      // bool empty = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          empty_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // int64 timestamp = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 32)) {
          timestamp_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag == 0) || ((tag & 7) == 4)) {
          CHK_(ptr);
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* State::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:state_representation.proto.State)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // string name = 1;
  if (!this->name().empty()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_name().data(), static_cast<int>(this->_internal_name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "state_representation.proto.State.name");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_name(), target);
  }

  // .state_representation.proto.StateType type = 2;
  if (this->type() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      2, this->_internal_type(), target);
  }

  // bool empty = 3;
  if (this->empty() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3, this->_internal_empty(), target);
  }

  // int64 timestamp = 4;
  if (this->timestamp() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt64ToArray(4, this->_internal_timestamp(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:state_representation.proto.State)
  return target;
}

size_t State::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:state_representation.proto.State)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // string name = 1;
  if (!this->name().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_name());
  }

  // .state_representation.proto.StateType type = 2;
  if (this->type() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_type());
  }

  // bool empty = 3;
  if (this->empty() != 0) {
    total_size += 1 + 1;
  }

  // int64 timestamp = 4;
  if (this->timestamp() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int64Size(
        this->_internal_timestamp());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void State::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:state_representation.proto.State)
  GOOGLE_DCHECK_NE(&from, this);
  const State* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<State>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:state_representation.proto.State)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:state_representation.proto.State)
    MergeFrom(*source);
  }
}

void State::MergeFrom(const State& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:state_representation.proto.State)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (!from.name().empty()) {
    _internal_set_name(from._internal_name());
  }
  if (from.type() != 0) {
    _internal_set_type(from._internal_type());
  }
  if (from.empty() != 0) {
    _internal_set_empty(from._internal_empty());
  }
  if (from.timestamp() != 0) {
    _internal_set_timestamp(from._internal_timestamp());
  }
}

void State::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:state_representation.proto.State)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void State::CopyFrom(const State& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:state_representation.proto.State)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool State::IsInitialized() const {
  return true;
}

void State::InternalSwap(State* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &name_, GetArenaForAllocation(),
      &other->name_, other->GetArenaForAllocation()
  );
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(State, timestamp_)
      + sizeof(State::timestamp_)
      - PROTOBUF_FIELD_OFFSET(State, type_)>(
          reinterpret_cast<char*>(&type_),
          reinterpret_cast<char*>(&other->type_));
}

::PROTOBUF_NAMESPACE_ID::Metadata State::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_state_5frepresentation_2fstate_2eproto_getter, &descriptor_table_state_5frepresentation_2fstate_2eproto_once,
      file_level_metadata_state_5frepresentation_2fstate_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace state_representation
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::state_representation::proto::State* Arena::CreateMaybeMessage< ::state_representation::proto::State >(Arena* arena) {
  return Arena::CreateMessageInternal< ::state_representation::proto::State >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>