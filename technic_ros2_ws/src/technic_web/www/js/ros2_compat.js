/*
 * ROS 2 compatibility for the Technic pages (load after roslib.js).
 *
 * ROS 2 has no global parameter server, and rosapi parameters belong to nodes,
 * so the pages' UI parameters (/navigate_ui/..., /land_ui/..., /genmap_runner/...)
 * are kept in the browser's localStorage with the same ROSLIB.Param API.
 */
(function () {
	if (typeof ROSLIB === 'undefined') return;

	var PREFIX = 'technic_param:';

	function LocalParam(options) {
		this.ros = options.ros;
		this.name = options.name;
	}

	LocalParam.prototype.get = function (callback, failedCallback) {
		var value = null;
		try {
			var raw = localStorage.getItem(PREFIX + this.name);
			value = raw === null ? null : JSON.parse(raw);
		} catch (error) {
			if (failedCallback) { failedCallback(String(error)); return; }
		}
		if (callback) callback(value);
	};

	LocalParam.prototype.set = function (value, callback, failedCallback) {
		try {
			localStorage.setItem(PREFIX + this.name, JSON.stringify(value));
		} catch (error) {
			// storage unavailable (private mode): the value only lives in the page
			if (failedCallback) { failedCallback(String(error)); return; }
		}
		if (callback) callback({});
	};

	LocalParam.prototype.delete = function (callback) {
		try { localStorage.removeItem(PREFIX + this.name); } catch (error) { /* ignore */ }
		if (callback) callback({});
	};

	ROSLIB.Param = LocalParam;
})();

/* mavros 2 runs as node "mavros" in namespace "mavros": topics are /mavros/mavros/... */
var TECHNIC_MAVROS = '/mavros/mavros';
