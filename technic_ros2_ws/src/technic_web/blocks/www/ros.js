/*
 * Copyright (C) 2020 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

var url = 'ws://' + location.hostname + ':9090';
export var ros = new ROSLIB.Ros({ url });

ros.on('connection', function () {
	console.log('connection')
	document.body.classList.add('connected');
});

ros.on('close', function () {
	console.log('close')
	document.body.classList.remove('connected');
	setTimeout(function() {
		ros.connect(url);
	}, 2000);
});

ros.on('error', function(err) {
});

export const namespace = '/';
export const priv = namespace + 'technic_blocks/';

export var params = {}; // parameters storage

// ROS 2 has no global parameters: technic_blocks returns its parameters as JSON
// from ~/get_config. URL parameters (?navigate_tolerance=0.5) still win.
var configPromise = null;

function readConfig() {
	if (!configPromise) {
		configPromise = new Promise(function(resolve) {
			new ROSLIB.Service({ ros: ros, name: priv + 'get_config', serviceType: 'std_srvs/srv/Trigger' })
				.callService(new ROSLIB.ServiceRequest(), function(res) {
					try {
						resolve(res.success ? JSON.parse(res.message) : {});
					} catch (e) {
						resolve({});
					}
				}, function() { resolve({}); });
			setTimeout(function() { resolve({}); }, 3000); // backend is not running
		});
	}
	return configPromise;
}

function parseValue(value, _default) {
	if (typeof _default === 'number') return Number(value);
	if (typeof _default === 'boolean') return !['false', '0', 'no', 'off'].includes(String(value).toLowerCase());
	return value;
}

export function readParam(name, fromUrl, _default) {
	if (params[name] !== undefined) return Promise.resolve();
	var url = fromUrl ? new URL(window.location.href).searchParams.get(name) : null;
	if (url !== null) {
		params[name] = parseValue(url, _default);
		return Promise.resolve();
	}
	return readConfig().then(function(config) {
		params[name] = config[name] !== undefined ? config[name] : _default;
	});
}


export var runService = new ROSLIB.Service({ ros: ros, name: priv + 'run', serviceType: 'technic_web/srv/Run' });
export var stopService = new ROSLIB.Service({ ros: ros, name: priv + 'stop', serviceType: 'std_srvs/srv/Trigger' });
export var loadService = new ROSLIB.Service({ ros: ros, name : priv + 'load', serviceType : 'technic_web/srv/Load' });
export var storeService = new ROSLIB.Service({ ros: ros, name : priv + 'store', serviceType : 'technic_web/srv/Store' });
export var landService = new ROSLIB.Service({ ros: ros, name : namespace + 'land', serviceType : 'std_srvs/srv/Trigger' });
