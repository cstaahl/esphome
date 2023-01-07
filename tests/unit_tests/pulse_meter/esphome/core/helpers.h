#pragma once

namespace esphome {
	/// Helper class to deduplicate items in a series of values.
	template<typename T> class Deduplicator {
	 public:
	  /// Feeds the next item in the series to the deduplicator and returns whether this is a duplicate.
	  bool next(T value) {
	    if (this->has_value_) {
	      if (this->last_value_ == value)
	        return false;
	    }
	    this->has_value_ = true;
	    this->last_value_ = value;
	    return true;
	  }
	  /// Returns whether this deduplicator has processed any items so far.
	  bool has_value() const { return this->has_value_; }

	 protected:
	  bool has_value_{false};
	  T last_value_{};
	};
}  // namespace esphome