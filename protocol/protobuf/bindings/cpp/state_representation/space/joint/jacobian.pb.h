// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: state_representation/space/joint/jacobian.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_state_5frepresentation_2fspace_2fjoint_2fjacobian_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_state_5frepresentation_2fspace_2fjoint_2fjacobian_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3017000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3017000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "state_representation/state.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_state_5frepresentation_2fspace_2fjoint_2fjacobian_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_state_5frepresentation_2fspace_2fjoint_2fjacobian_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_state_5frepresentation_2fspace_2fjoint_2fjacobian_2eproto;
namespace state_representation {
namespace proto {
class Jacobian;
struct JacobianDefaultTypeInternal;
extern JacobianDefaultTypeInternal _Jacobian_default_instance_;
}  // namespace proto
}  // namespace state_representation
PROTOBUF_NAMESPACE_OPEN
template<> ::state_representation::proto::Jacobian* Arena::CreateMaybeMessage<::state_representation::proto::Jacobian>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace state_representation {
namespace proto {

// ===================================================================

class Jacobian final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:state_representation.proto.Jacobian) */ {
 public:
  inline Jacobian() : Jacobian(nullptr) {}
  ~Jacobian() override;
  explicit constexpr Jacobian(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Jacobian(const Jacobian& from);
  Jacobian(Jacobian&& from) noexcept
    : Jacobian() {
    *this = ::std::move(from);
  }

  inline Jacobian& operator=(const Jacobian& from) {
    CopyFrom(from);
    return *this;
  }
  inline Jacobian& operator=(Jacobian&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Jacobian& default_instance() {
    return *internal_default_instance();
  }
  static inline const Jacobian* internal_default_instance() {
    return reinterpret_cast<const Jacobian*>(
               &_Jacobian_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Jacobian& a, Jacobian& b) {
    a.Swap(&b);
  }
  inline void Swap(Jacobian* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Jacobian* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Jacobian* New() const final {
    return new Jacobian();
  }

  Jacobian* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Jacobian>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Jacobian& from);
  void MergeFrom(const Jacobian& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Jacobian* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "state_representation.proto.Jacobian";
  }
  protected:
  explicit Jacobian(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kJointNamesFieldNumber = 2,
    kDataFieldNumber = 7,
    kFrameFieldNumber = 3,
    kReferenceFrameFieldNumber = 4,
    kStateFieldNumber = 1,
    kRowsFieldNumber = 5,
    kColsFieldNumber = 6,
  };
  // repeated string joint_names = 2;
  int joint_names_size() const;
  private:
  int _internal_joint_names_size() const;
  public:
  void clear_joint_names();
  const std::string& joint_names(int index) const;
  std::string* mutable_joint_names(int index);
  void set_joint_names(int index, const std::string& value);
  void set_joint_names(int index, std::string&& value);
  void set_joint_names(int index, const char* value);
  void set_joint_names(int index, const char* value, size_t size);
  std::string* add_joint_names();
  void add_joint_names(const std::string& value);
  void add_joint_names(std::string&& value);
  void add_joint_names(const char* value);
  void add_joint_names(const char* value, size_t size);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>& joint_names() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>* mutable_joint_names();
  private:
  const std::string& _internal_joint_names(int index) const;
  std::string* _internal_add_joint_names();
  public:

  // repeated double data = 7;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  private:
  double _internal_data(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_data() const;
  void _internal_add_data(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_data();
  public:
  double data(int index) const;
  void set_data(int index, double value);
  void add_data(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      data() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_data();

  // string frame = 3;
  void clear_frame();
  const std::string& frame() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_frame(ArgT0&& arg0, ArgT... args);
  std::string* mutable_frame();
  PROTOBUF_FUTURE_MUST_USE_RESULT std::string* release_frame();
  void set_allocated_frame(std::string* frame);
  private:
  const std::string& _internal_frame() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_frame(const std::string& value);
  std::string* _internal_mutable_frame();
  public:

  // string reference_frame = 4;
  void clear_reference_frame();
  const std::string& reference_frame() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_reference_frame(ArgT0&& arg0, ArgT... args);
  std::string* mutable_reference_frame();
  PROTOBUF_FUTURE_MUST_USE_RESULT std::string* release_reference_frame();
  void set_allocated_reference_frame(std::string* reference_frame);
  private:
  const std::string& _internal_reference_frame() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_reference_frame(const std::string& value);
  std::string* _internal_mutable_reference_frame();
  public:

  // .state_representation.proto.State state = 1;
  bool has_state() const;
  private:
  bool _internal_has_state() const;
  public:
  void clear_state();
  const ::state_representation::proto::State& state() const;
  PROTOBUF_FUTURE_MUST_USE_RESULT ::state_representation::proto::State* release_state();
  ::state_representation::proto::State* mutable_state();
  void set_allocated_state(::state_representation::proto::State* state);
  private:
  const ::state_representation::proto::State& _internal_state() const;
  ::state_representation::proto::State* _internal_mutable_state();
  public:
  void unsafe_arena_set_allocated_state(
      ::state_representation::proto::State* state);
  ::state_representation::proto::State* unsafe_arena_release_state();

  // uint32 rows = 5;
  void clear_rows();
  ::PROTOBUF_NAMESPACE_ID::uint32 rows() const;
  void set_rows(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_rows() const;
  void _internal_set_rows(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // uint32 cols = 6;
  void clear_cols();
  ::PROTOBUF_NAMESPACE_ID::uint32 cols() const;
  void set_cols(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_cols() const;
  void _internal_set_cols(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:state_representation.proto.Jacobian)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string> joint_names_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > data_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr frame_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr reference_frame_;
  ::state_representation::proto::State* state_;
  ::PROTOBUF_NAMESPACE_ID::uint32 rows_;
  ::PROTOBUF_NAMESPACE_ID::uint32 cols_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_state_5frepresentation_2fspace_2fjoint_2fjacobian_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Jacobian

// .state_representation.proto.State state = 1;
inline bool Jacobian::_internal_has_state() const {
  return this != internal_default_instance() && state_ != nullptr;
}
inline bool Jacobian::has_state() const {
  return _internal_has_state();
}
inline const ::state_representation::proto::State& Jacobian::_internal_state() const {
  const ::state_representation::proto::State* p = state_;
  return p != nullptr ? *p : reinterpret_cast<const ::state_representation::proto::State&>(
      ::state_representation::proto::_State_default_instance_);
}
inline const ::state_representation::proto::State& Jacobian::state() const {
  // @@protoc_insertion_point(field_get:state_representation.proto.Jacobian.state)
  return _internal_state();
}
inline void Jacobian::unsafe_arena_set_allocated_state(
    ::state_representation::proto::State* state) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(state_);
  }
  state_ = state;
  if (state) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:state_representation.proto.Jacobian.state)
}
inline ::state_representation::proto::State* Jacobian::release_state() {
  
  ::state_representation::proto::State* temp = state_;
  state_ = nullptr;
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::state_representation::proto::State* Jacobian::unsafe_arena_release_state() {
  // @@protoc_insertion_point(field_release:state_representation.proto.Jacobian.state)
  
  ::state_representation::proto::State* temp = state_;
  state_ = nullptr;
  return temp;
}
inline ::state_representation::proto::State* Jacobian::_internal_mutable_state() {
  
  if (state_ == nullptr) {
    auto* p = CreateMaybeMessage<::state_representation::proto::State>(GetArenaForAllocation());
    state_ = p;
  }
  return state_;
}
inline ::state_representation::proto::State* Jacobian::mutable_state() {
  // @@protoc_insertion_point(field_mutable:state_representation.proto.Jacobian.state)
  return _internal_mutable_state();
}
inline void Jacobian::set_allocated_state(::state_representation::proto::State* state) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(state_);
  }
  if (state) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(state));
    if (message_arena != submessage_arena) {
      state = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, state, submessage_arena);
    }
    
  } else {
    
  }
  state_ = state;
  // @@protoc_insertion_point(field_set_allocated:state_representation.proto.Jacobian.state)
}

// repeated string joint_names = 2;
inline int Jacobian::_internal_joint_names_size() const {
  return joint_names_.size();
}
inline int Jacobian::joint_names_size() const {
  return _internal_joint_names_size();
}
inline void Jacobian::clear_joint_names() {
  joint_names_.Clear();
}
inline std::string* Jacobian::add_joint_names() {
  // @@protoc_insertion_point(field_add_mutable:state_representation.proto.Jacobian.joint_names)
  return _internal_add_joint_names();
}
inline const std::string& Jacobian::_internal_joint_names(int index) const {
  return joint_names_.Get(index);
}
inline const std::string& Jacobian::joint_names(int index) const {
  // @@protoc_insertion_point(field_get:state_representation.proto.Jacobian.joint_names)
  return _internal_joint_names(index);
}
inline std::string* Jacobian::mutable_joint_names(int index) {
  // @@protoc_insertion_point(field_mutable:state_representation.proto.Jacobian.joint_names)
  return joint_names_.Mutable(index);
}
inline void Jacobian::set_joint_names(int index, const std::string& value) {
  // @@protoc_insertion_point(field_set:state_representation.proto.Jacobian.joint_names)
  joint_names_.Mutable(index)->assign(value);
}
inline void Jacobian::set_joint_names(int index, std::string&& value) {
  // @@protoc_insertion_point(field_set:state_representation.proto.Jacobian.joint_names)
  joint_names_.Mutable(index)->assign(std::move(value));
}
inline void Jacobian::set_joint_names(int index, const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  joint_names_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set_char:state_representation.proto.Jacobian.joint_names)
}
inline void Jacobian::set_joint_names(int index, const char* value, size_t size) {
  joint_names_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:state_representation.proto.Jacobian.joint_names)
}
inline std::string* Jacobian::_internal_add_joint_names() {
  return joint_names_.Add();
}
inline void Jacobian::add_joint_names(const std::string& value) {
  joint_names_.Add()->assign(value);
  // @@protoc_insertion_point(field_add:state_representation.proto.Jacobian.joint_names)
}
inline void Jacobian::add_joint_names(std::string&& value) {
  joint_names_.Add(std::move(value));
  // @@protoc_insertion_point(field_add:state_representation.proto.Jacobian.joint_names)
}
inline void Jacobian::add_joint_names(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  joint_names_.Add()->assign(value);
  // @@protoc_insertion_point(field_add_char:state_representation.proto.Jacobian.joint_names)
}
inline void Jacobian::add_joint_names(const char* value, size_t size) {
  joint_names_.Add()->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_add_pointer:state_representation.proto.Jacobian.joint_names)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>&
Jacobian::joint_names() const {
  // @@protoc_insertion_point(field_list:state_representation.proto.Jacobian.joint_names)
  return joint_names_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>*
Jacobian::mutable_joint_names() {
  // @@protoc_insertion_point(field_mutable_list:state_representation.proto.Jacobian.joint_names)
  return &joint_names_;
}

// string frame = 3;
inline void Jacobian::clear_frame() {
  frame_.ClearToEmpty();
}
inline const std::string& Jacobian::frame() const {
  // @@protoc_insertion_point(field_get:state_representation.proto.Jacobian.frame)
  return _internal_frame();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void Jacobian::set_frame(ArgT0&& arg0, ArgT... args) {
 
 frame_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:state_representation.proto.Jacobian.frame)
}
inline std::string* Jacobian::mutable_frame() {
  // @@protoc_insertion_point(field_mutable:state_representation.proto.Jacobian.frame)
  return _internal_mutable_frame();
}
inline const std::string& Jacobian::_internal_frame() const {
  return frame_.Get();
}
inline void Jacobian::_internal_set_frame(const std::string& value) {
  
  frame_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* Jacobian::_internal_mutable_frame() {
  
  return frame_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* Jacobian::release_frame() {
  // @@protoc_insertion_point(field_release:state_representation.proto.Jacobian.frame)
  return frame_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void Jacobian::set_allocated_frame(std::string* frame) {
  if (frame != nullptr) {
    
  } else {
    
  }
  frame_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), frame,
      GetArenaForAllocation());
  // @@protoc_insertion_point(field_set_allocated:state_representation.proto.Jacobian.frame)
}

// string reference_frame = 4;
inline void Jacobian::clear_reference_frame() {
  reference_frame_.ClearToEmpty();
}
inline const std::string& Jacobian::reference_frame() const {
  // @@protoc_insertion_point(field_get:state_representation.proto.Jacobian.reference_frame)
  return _internal_reference_frame();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void Jacobian::set_reference_frame(ArgT0&& arg0, ArgT... args) {
 
 reference_frame_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:state_representation.proto.Jacobian.reference_frame)
}
inline std::string* Jacobian::mutable_reference_frame() {
  // @@protoc_insertion_point(field_mutable:state_representation.proto.Jacobian.reference_frame)
  return _internal_mutable_reference_frame();
}
inline const std::string& Jacobian::_internal_reference_frame() const {
  return reference_frame_.Get();
}
inline void Jacobian::_internal_set_reference_frame(const std::string& value) {
  
  reference_frame_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* Jacobian::_internal_mutable_reference_frame() {
  
  return reference_frame_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* Jacobian::release_reference_frame() {
  // @@protoc_insertion_point(field_release:state_representation.proto.Jacobian.reference_frame)
  return reference_frame_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void Jacobian::set_allocated_reference_frame(std::string* reference_frame) {
  if (reference_frame != nullptr) {
    
  } else {
    
  }
  reference_frame_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), reference_frame,
      GetArenaForAllocation());
  // @@protoc_insertion_point(field_set_allocated:state_representation.proto.Jacobian.reference_frame)
}

// uint32 rows = 5;
inline void Jacobian::clear_rows() {
  rows_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Jacobian::_internal_rows() const {
  return rows_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Jacobian::rows() const {
  // @@protoc_insertion_point(field_get:state_representation.proto.Jacobian.rows)
  return _internal_rows();
}
inline void Jacobian::_internal_set_rows(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  
  rows_ = value;
}
inline void Jacobian::set_rows(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_rows(value);
  // @@protoc_insertion_point(field_set:state_representation.proto.Jacobian.rows)
}

// uint32 cols = 6;
inline void Jacobian::clear_cols() {
  cols_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Jacobian::_internal_cols() const {
  return cols_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Jacobian::cols() const {
  // @@protoc_insertion_point(field_get:state_representation.proto.Jacobian.cols)
  return _internal_cols();
}
inline void Jacobian::_internal_set_cols(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  
  cols_ = value;
}
inline void Jacobian::set_cols(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_cols(value);
  // @@protoc_insertion_point(field_set:state_representation.proto.Jacobian.cols)
}

// repeated double data = 7;
inline int Jacobian::_internal_data_size() const {
  return data_.size();
}
inline int Jacobian::data_size() const {
  return _internal_data_size();
}
inline void Jacobian::clear_data() {
  data_.Clear();
}
inline double Jacobian::_internal_data(int index) const {
  return data_.Get(index);
}
inline double Jacobian::data(int index) const {
  // @@protoc_insertion_point(field_get:state_representation.proto.Jacobian.data)
  return _internal_data(index);
}
inline void Jacobian::set_data(int index, double value) {
  data_.Set(index, value);
  // @@protoc_insertion_point(field_set:state_representation.proto.Jacobian.data)
}
inline void Jacobian::_internal_add_data(double value) {
  data_.Add(value);
}
inline void Jacobian::add_data(double value) {
  _internal_add_data(value);
  // @@protoc_insertion_point(field_add:state_representation.proto.Jacobian.data)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
Jacobian::_internal_data() const {
  return data_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
Jacobian::data() const {
  // @@protoc_insertion_point(field_list:state_representation.proto.Jacobian.data)
  return _internal_data();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
Jacobian::_internal_mutable_data() {
  return &data_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
Jacobian::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:state_representation.proto.Jacobian.data)
  return _internal_mutable_data();
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace state_representation

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_state_5frepresentation_2fspace_2fjoint_2fjacobian_2eproto