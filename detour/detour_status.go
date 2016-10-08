package detour

type statusError uint

func (e statusError) Error() string {
	if e.flag(DT_WRONG_MAGIC) {
		return "wrong magic"
	} else if e.flag(DT_WRONG_VERSION) {
		return "wrong version"
	} else if e.flag(DT_INVALID_PARAM) {
		return "invalid parameter"
	} else if e.flag(DT_BUFFER_TOO_SMALL) {
		return "buffer too small"
	} else if e.flag(DT_OUT_OF_NODES) {
		return "out of nodes"
	} else if e.flag(DT_PARTIAL_RESULT) {
		return "partial result"
	}

	if e.flag(DT_FAILURE) {
		return "failure"
	} else if e.flag(DT_IN_PROGRESS) {
		return "in progress"
	}

	return ""
}

func (e statusError) flag(bit uint) bool {
	return (uint(e) | bit) != 0
}

// High level status.
const (
	DT_FAILURE     = 1 << 31 // Operation failed.
	DT_SUCCESS     = 1 << 30 // Operation succeed.
	DT_IN_PROGRESS = 1 << 29 // Operation still in progress.
)

// Detail information for status.
const (
	DT_STATUS_DETAIL_MASK = 0x0ffffff
	DT_WRONG_MAGIC        = 1 << 0 // Input data is not recognized.
	DT_WRONG_VERSION      = 1 << 1 // Input data is in wrong version.
	DT_OUT_OF_MEMORY      = 1 << 2 // Operation ran out of memory.
	DT_INVALID_PARAM      = 1 << 3 // An input parameter was invalid.
	DT_BUFFER_TOO_SMALL   = 1 << 4 // Result buffer for the query was too small to store all results.
	DT_OUT_OF_NODES       = 1 << 5 // Query ran out of nodes during search.
	DT_PARTIAL_RESULT     = 1 << 6 // Query did not reach the end location, returning best guess.
)

// Returns true of status is success.
func StatusSucceed(err error) bool {
	if err == nil {
		return true
	}

	status, ok := err.(statusError)
	return ok && ((status & DT_SUCCESS) != 0)
}

// Returns true of status is failure.
func StatusFailed(err error) bool {
	status, ok := err.(statusError)
	return ok && ((status & DT_FAILURE) != 0)
}

// Returns true of status is in progress.
func StatusInProgress(err error) bool {
	status, ok := err.(statusError)
	return ok && ((status & DT_IN_PROGRESS) != 0)
}

// Returns true if specific detail is set.
func StatusDetail(err error, detail uint) bool {
	status, ok := err.(statusError)
	return ok && ((uint(status) & detail) != 0)
}
