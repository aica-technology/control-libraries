// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: state_representation/geometry/shape.proto

#include "state_representation/geometry/shape.pb.h"

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
constexpr Shape::Shape(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : state_(nullptr)
  , center_(nullptr){}
struct ShapeDefaultTypeInternal {
  constexpr ShapeDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~ShapeDefaultTypeInternal() {}
  union {
    Shape _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT ShapeDefaultTypeInternal _Shape_default_instance_;
}  // namespace proto
}  // namespace state_representation
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_state_5frepresentation_2fgeometry_2fshape_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_state_5frepresentation_2fgeometry_2fshape_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_state_5frepresentation_2fgeometry_2fshape_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_state_5frepresentation_2fgeometry_2fshape_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::state_representation::proto::Shape, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::state_representation::proto::Shape, state_),
  PROTOBUF_FIELD_OFFSET(::state_representation::proto::Shape, center_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::state_representation::proto::Shape)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::state_representation::proto::_Shape_default_instance_),
};

const char descriptor_table_protodef_state_5frepresentation_2fgeometry_2fshape_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n)state_representation/geometry/shape.pr"
  "oto\022\032state_representation.proto\032 state_r"
  "epresentation/state.proto\032:state_represe"
  "ntation/space/cartesian/cartesian_state."
  "proto\"u\n\005Shape\0220\n\005state\030\001 \001(\0132!.state_re"
  "presentation.proto.State\022:\n\006center\030\002 \001(\013"
  "2*.state_representation.proto.CartesianS"
  "tateb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_state_5frepresentation_2fgeometry_2fshape_2eproto_deps[2] = {
  &::descriptor_table_state_5frepresentation_2fspace_2fcartesian_2fcartesian_5fstate_2eproto,
  &::descriptor_table_state_5frepresentation_2fstate_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_state_5frepresentation_2fgeometry_2fshape_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_state_5frepresentation_2fgeometry_2fshape_2eproto = {
  false, false, 292, descriptor_table_protodef_state_5frepresentation_2fgeometry_2fshape_2eproto, "state_representation/geometry/shape.proto", 
  &descriptor_table_state_5frepresentation_2fgeometry_2fshape_2eproto_once, descriptor_table_state_5frepresentation_2fgeometry_2fshape_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_state_5frepresentation_2fgeometry_2fshape_2eproto::offsets,
  file_level_metadata_state_5frepresentation_2fgeometry_2fshape_2eproto, file_level_enum_descriptors_state_5frepresentation_2fgeometry_2fshape_2eproto, file_level_service_descriptors_state_5frepresentation_2fgeometry_2fshape_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_state_5frepresentation_2fgeometry_2fshape_2eproto_getter() {
  return &descriptor_table_state_5frepresentation_2fgeometry_2fshape_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_state_5frepresentation_2fgeometry_2fshape_2eproto(&descriptor_table_state_5frepresentation_2fgeometry_2fshape_2eproto);
namespace state_representation {
namespace proto {

// ===================================================================

class Shape::_Internal {
 public:
  static const ::state_representation::proto::State& state(const Shape* msg);
  static const ::state_representation::proto::CartesianState& center(const Shape* msg);
};

const ::state_representation::proto::State&
Shape::_Internal::state(const Shape* msg) {
  return *msg->state_;
}
const ::state_representation::proto::CartesianState&
Shape::_Internal::center(const Shape* msg) {
  return *msg->center_;
}
void Shape::clear_state() {
  if (GetArenaForAllocation() == nullptr && state_ != nullptr) {
    delete state_;
  }
  state_ = nullptr;
}
void Shape::clear_center() {
  if (GetArenaForAllocation() == nullptr && center_ != nullptr) {
    delete center_;
  }
  center_ = nullptr;
}
Shape::Shape(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:state_representation.proto.Shape)
}
Shape::Shape(const Shape& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_state()) {
    state_ = new ::state_representation::proto::State(*from.state_);
  } else {
    state_ = nullptr;
  }
  if (from._internal_has_center()) {
    center_ = new ::state_representation::proto::CartesianState(*from.center_);
  } else {
    center_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:state_representation.proto.Shape)
}

void Shape::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&state_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&center_) -
    reinterpret_cast<char*>(&state_)) + sizeof(center_));
}

Shape::~Shape() {
  // @@protoc_insertion_point(destructor:state_representation.proto.Shape)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void Shape::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete state_;
  if (this != internal_default_instance()) delete center_;
}

void Shape::ArenaDtor(void* object) {
  Shape* _this = reinterpret_cast< Shape* >(object);
  (void)_this;
}
void Shape::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Shape::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Shape::Clear() {
// @@protoc_insertion_point(message_clear_start:state_representation.proto.Shape)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && state_ != nullptr) {
    delete state_;
  }
  state_ = nullptr;
  if (GetArenaForAllocation() == nullptr && center_ != nullptr) {
    delete center_;
  }
  center_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Shape::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .state_representation.proto.State state = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_state(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // .state_representation.proto.CartesianState center = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_center(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* Shape::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:state_representation.proto.Shape)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .state_representation.proto.State state = 1;
  if (this->has_state()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::state(this), target, stream);
  }

  // .state_representation.proto.CartesianState center = 2;
  if (this->has_center()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::center(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:state_representation.proto.Shape)
  return target;
}

size_t Shape::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:state_representation.proto.Shape)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .state_representation.proto.State state = 1;
  if (this->has_state()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *state_);
  }

  // .state_representation.proto.CartesianState center = 2;
  if (this->has_center()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *center_);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Shape::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:state_representation.proto.Shape)
  GOOGLE_DCHECK_NE(&from, this);
  const Shape* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Shape>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:state_representation.proto.Shape)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:state_representation.proto.Shape)
    MergeFrom(*source);
  }
}

void Shape::MergeFrom(const Shape& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:state_representation.proto.Shape)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_state()) {
    _internal_mutable_state()->::state_representation::proto::State::MergeFrom(from._internal_state());
  }
  if (from.has_center()) {
    _internal_mutable_center()->::state_representation::proto::CartesianState::MergeFrom(from._internal_center());
  }
}

void Shape::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:state_representation.proto.Shape)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Shape::CopyFrom(const Shape& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:state_representation.proto.Shape)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Shape::IsInitialized() const {
  return true;
}

void Shape::InternalSwap(Shape* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Shape, center_)
      + sizeof(Shape::center_)
      - PROTOBUF_FIELD_OFFSET(Shape, state_)>(
          reinterpret_cast<char*>(&state_),
          reinterpret_cast<char*>(&other->state_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Shape::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_state_5frepresentation_2fgeometry_2fshape_2eproto_getter, &descriptor_table_state_5frepresentation_2fgeometry_2fshape_2eproto_once,
      file_level_metadata_state_5frepresentation_2fgeometry_2fshape_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace state_representation
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::state_representation::proto::Shape* Arena::CreateMaybeMessage< ::state_representation::proto::Shape >(Arena* arena) {
  return Arena::CreateMessageInternal< ::state_representation::proto::Shape >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>