import {params} from './ros.js';

// ROS 2: programs call technic_blocks.api (see technic_web/technic_blocks/api.py)
// instead of rospy service proxies of simple_offboard; the API talks to the
// web_api services /navigate, /get_telemetry, /land, /set_velocity.

// If any new block imports any library, add that library name here.
Blockly.Python.addReservedWords('_b,_print');
Blockly.Python.addReservedWords('get_telemetry,navigate,navigate_global,set_velocity,set_yaw,set_attitude,set_rates,land');
Blockly.Python.addReservedWords('sleep,get_time,is_shutdown,wait_for_message,get_range,get_rc_channel');
Blockly.Python.addReservedWords('navigate_wait,navigate_global_wait,land_wait,wait_arrival,wait_yaw,arrived,get_distance');
Blockly.Python.addReservedWords('set_effect,set_leds,LEDState,led_count,get_led_count');
Blockly.Python.addReservedWords('gpio_read,gpio_write,set_servo,set_duty_cycle');

const NAVIGATE_WAIT = () => `\ndef navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        raise Exception(res.message)

    while not is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_tolerance}:
            return
        sleep(${params.sleep_time})\n`;

const NAVIGATE_GLOBAL_WAIT = () => `\ndef navigate_global_wait(lat, lon, z, speed=0.5):
    res = navigate_global(lat=lat, lon=lon, z=z, yaw=float('inf'), speed=speed)

    if not res.success:
        raise Exception(res.message)

    while not is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_global_tolerance}:
            return
        sleep(${params.sleep_time})\n`;

const LAND_WAIT = () => `\ndef land_wait():
    res = land()

    if not res.success:
        raise Exception(res.message)

    while get_telemetry().armed:
        sleep(${params.sleep_time})\n`;

const WAIT_YAW = () => `\ndef wait_yaw():
    while not is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if abs(telem.yaw) < math.radians(${params.yaw_tolerance}):
            return
        sleep(${params.sleep_time})\n`;

const WAIT_ARRIVAL = () => `\ndef wait_arrival():
    while not is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_tolerance}:
            return
        sleep(${params.sleep_time})\n`;

const ARRIVED = () => `\ndef arrived():
    telem = get_telemetry(frame_id='navigate_target')
    return math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_tolerance}\n`

const GET_DISTANCE = `\ndef get_distance(x, y, z, frame_id):
    telem = get_telemetry(frame_id)
    return math.sqrt((x - telem.x) ** 2 + (y - telem.y) ** 2 + (z - telem.z) ** 2)\n`;

var rosDefinitions = {};
var apiImports = new Set();

// names imported from technic_blocks.api, in a stable order
const API_ORDER = ['get_telemetry', 'navigate', 'navigate_global', 'set_yaw', 'set_velocity',
	'set_attitude', 'set_rates', 'land', 'sleep', 'get_time', 'is_shutdown', 'get_range',
	'get_rc_channel', 'set_effect', 'set_leds', 'LEDState', 'get_led_count'];

function useApi(...names) {
	for (let name of names) apiImports.add(name);
	let list = API_ORDER.filter(n => apiImports.has(n));
	Blockly.Python.definitions_['import_api'] = 'from technic_blocks.api import ' + list.join(', ');
}

function useMath() {
	Blockly.Python.definitions_['import_math'] = 'import math';
}

function generateROSDefinitions() {
	// helper functions depend on each other, so generate them all as one definition
	var code = '';
	if (rosDefinitions.navigateWait) {
		useMath();
		useApi('navigate', 'get_telemetry', 'is_shutdown', 'sleep');
		code += NAVIGATE_WAIT();
	}
	if (rosDefinitions.navigateGlobalWait) {
		useMath();
		useApi('navigate_global', 'get_telemetry', 'is_shutdown', 'sleep');
		code += NAVIGATE_GLOBAL_WAIT();
	}
	if (rosDefinitions.landWait) {
		useApi('land', 'get_telemetry', 'sleep');
		code += LAND_WAIT();
	}
	if (rosDefinitions.waitArrival) {
		useMath();
		useApi('get_telemetry', 'is_shutdown', 'sleep');
		code += WAIT_ARRIVAL();
	}
	if (rosDefinitions.arrived) {
		useMath();
		useApi('get_telemetry');
		code += ARRIVED();
	}
	if (rosDefinitions.waitYaw) {
		useMath();
		useApi('get_telemetry', 'is_shutdown', 'sleep');
		code += WAIT_YAW();
	}
	if (rosDefinitions.distance) {
		useMath();
		useApi('get_telemetry');
		code += GET_DISTANCE;
	}
	if (code) {
		Blockly.Python.definitions_['ros'] = code;
	}
}

// kept with the ROS 1 names: blocks call these before generating their code
function initNode(...names) {
	useApi(...names);
	generateROSDefinitions();
}

function simpleOffboard(...names) {
	initNode(...names);
}

// Adjust indentation
Blockly.Python.INDENT = '    ';

export function generateUserCode(workspace) {
	rosDefinitions = {};
	apiImports = new Set();
	Blockly.Python.STATEMENT_PREFIX = null;
	return Blockly.Python.workspaceToCode(workspace);
}

export function generateCode(workspace) {
	rosDefinitions = {};
	apiImports = new Set();
	Blockly.Python.STATEMENT_PREFIX = '_b(%1)\n';
	var code = Blockly.Python.workspaceToCode(workspace);
	return code;
}

function buildFrameId(block) {
	let frame = block.getFieldValue('FRAME_ID').toLowerCase();
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	if (frame == 'aruco') { // aruco marker frame
		if (id.match(/^[0-9]+$/)) { // id is positive integer
			return `'${frame}_${id}'`;
		} else { // something else...
			return `'${frame}_' + str(int(${id}))`;
		}
	} else {
		return `'${frame}'`;
	}
}

Blockly.Python.navigate = function(block) {
	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let lat = Blockly.Python.valueToCode(block, 'LAT', Blockly.Python.ORDER_NONE);
	let lon = Blockly.Python.valueToCode(block, 'LON', Blockly.Python.ORDER_NONE);
	let wait = block.getFieldValue('WAIT') == 'TRUE';
	let frameId = block.getFieldValue('FRAME_ID');
	let speed = Blockly.Python.valueToCode(block, 'SPEED', Blockly.Python.ORDER_NONE);

	// global coordinates
	if (frameId.startsWith('GLOBAL')) {
		if (frameId == 'GLOBAL') {
			simpleOffboard('get_telemetry');
			z = `${z} + get_telemetry().alt - get_telemetry().z`;
		}

		if (wait) {
			rosDefinitions.navigateGlobalWait = true;
			simpleOffboard();
			return `navigate_global_wait(lat=${lat}, lon=${lon}, z=${z}, speed=${speed})\n`;

		} else {
			simpleOffboard('navigate_global');
			return `navigate_global(lat=${lat}, lon=${lon}, z=${z}, yaw=float('inf'), speed=${speed})\n`;
		}

	} else {
		frameId = buildFrameId(block);
		let params = [`x=${x}`, `y=${y}`, `z=${z}`, `frame_id=${frameId}`, `speed=${speed}`];

		if (wait) {
			rosDefinitions.navigateWait = true;
			simpleOffboard();

			return `navigate_wait(${params.join(', ')})\n`;

		} else {
			simpleOffboard('navigate');
			params.push(`yaw=float('nan')`); // keep the current yaw, as ROS 1 blocks did
			return `navigate(${params.join(', ')})\n`;
		}
	}
}

Blockly.Python.set_velocity = function(block) {
	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);

	simpleOffboard('set_velocity');

	if (frameId == `'body'`) {
		return `set_velocity(vx=${x}, vy=${y}, vz=${z}, frame_id=${frameId})\n`;
	} else {
		return `set_velocity(vx=${x}, vy=${y}, vz=${z}, yaw=float('nan'), frame_id=${frameId})\n`;
	}
}

Blockly.Python.take_off = function(block) {
	let z = Blockly.Python.valueToCode(block, 'ALT', Blockly.Python.ORDER_NONE);

	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.navigateWait = true;
		simpleOffboard();

		return `navigate_wait(z=${z}, frame_id='body', auto_arm=True)\n`;
	} else {
		simpleOffboard('navigate');
		return `navigate(z=${z}, frame_id='body', auto_arm=True)\n`;
	}
}

Blockly.Python.land = function(block) {
	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.landWait = true;
		simpleOffboard();

		return `land_wait()\n`;
	} else {
		simpleOffboard('land');
		return 'land()\n';
	}
}

Blockly.Python.angle = function(block) {
	useMath();
	return [`math.radians(${block.getFieldValue('ANGLE')})`, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.set_yaw = function(block) {
	simpleOffboard('set_yaw');
	let yaw = Blockly.Python.valueToCode(block, 'YAW', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);
	let code = `set_yaw(yaw=${yaw}, frame_id=${frameId})\n`;
	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.waitYaw = true;
		simpleOffboard();
		code += 'wait_yaw()\n';
	}
	return code;
}

Blockly.Python.wait_arrival = function(block) {
	rosDefinitions.waitArrival = true;
	simpleOffboard();
	return 'wait_arrival()\n';
}

Blockly.Python.get_time = function(block) {
	initNode('get_time');
	return ['get_time()', Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.arrived = function(block) {
	rosDefinitions.arrived = true;
	simpleOffboard();
	return ['arrived()', Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.wait = function(block) {
	initNode('sleep');
	return `sleep(${Blockly.Python.valueToCode(block, 'TIME', Blockly.Python.ORDER_NONE)})\n`;
}

Blockly.Python.setpoint = function(block) {
	var type = block.getFieldValue('TYPE');
	let frameId = buildFrameId(block);
	let vx = Blockly.Python.valueToCode(block, 'VX', Blockly.Python.ORDER_NONE);
	let vy = Blockly.Python.valueToCode(block, 'VY', Blockly.Python.ORDER_NONE);
	let vz = Blockly.Python.valueToCode(block, 'VZ', Blockly.Python.ORDER_NONE);
	let yaw = Blockly.Python.valueToCode(block, 'YAW', Blockly.Python.ORDER_NONE);
	let pitch = Blockly.Python.valueToCode(block, 'PITCH', Blockly.Python.ORDER_NONE);
	let roll = Blockly.Python.valueToCode(block, 'ROLL', Blockly.Python.ORDER_NONE);
	let thrust = Blockly.Python.valueToCode(block, 'THRUST', Blockly.Python.ORDER_NONE);

	if (type == 'VELOCITY') {
		simpleOffboard('set_velocity');
		return `set_velocity(vx=${vx}, vy=${vy}, vz=${vz}, frame_id=${frameId}, yaw=float('nan'))\n`;
	} else if (type == 'ATTITUDE') {
		simpleOffboard('set_attitude');
		return `set_attitude(roll=${roll}, pitch=${pitch}, yaw=${yaw}, thrust=${thrust}, frame_id=${frameId})\n`;
	} else if (type == 'RATES') {
		simpleOffboard('set_rates');
		return `set_rates(roll_rate=${roll}, pitch_rate=${pitch}, yaw_rate=${yaw}, thrust=${thrust})\n`;
	}
}

Blockly.Python.get_position = function(block) {
	simpleOffboard('get_telemetry');
	let frameId = buildFrameId(block);
	var code = `get_telemetry(${frameId}).${block.getFieldValue('FIELD').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.get_yaw = function(block) {
	simpleOffboard('get_telemetry');
	useMath();
	let frameId = buildFrameId(block);
	var code = `math.degrees(get_telemetry(${frameId}).yaw)`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.get_attitude = function(block) {
	simpleOffboard('get_telemetry');
	useMath();
	var code = `math.degrees(get_telemetry().${block.getFieldValue('FIELD').toLowerCase()})`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.global_position = function(block) {
	simpleOffboard('get_telemetry');
	var code = `get_telemetry().${block.getFieldValue('FIELD').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.distance = function(block) {
	rosDefinitions.distance = true;
	simpleOffboard();

	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);

	return [`get_distance(${x}, ${y}, ${z}, ${frameId})`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.rangefinder_distance = function(block) {
	// vl53l1x node "rangefinder", topic /rangefinder/range
	initNode('get_range');
	return [`get_range()`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.mode = function(block) {
	simpleOffboard('get_telemetry');
	return [`get_telemetry().mode`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.armed = function(block) {
	simpleOffboard('get_telemetry');
	return [`get_telemetry().armed`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.voltage = function(block) {
	simpleOffboard('get_telemetry');
	var code = `get_telemetry().${block.getFieldValue('TYPE').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.get_rc = function(block) {
	// mavros /mavros/mavros/rc/in
	initNode('get_rc_channel');
	var channel = Blockly.Python.valueToCode(block, 'CHANNEL', Blockly.Python.ORDER_NONE);
	return [`get_rc_channel(${channel})`, Blockly.Python.ORDER_FUNCTION_CALL]
}

function parseColor(color) {
	return {
		r: parseInt(color.substr(2, 2), 16),
		g: parseInt(color.substr(4, 2), 16),
		b: parseInt(color.substr(6, 2), 16)
	}
}

const PARSE_COLOR = `def ${Blockly.Python.FUNCTION_NAME_PLACEHOLDER_}(color):
  return {'r': int(color[1:3], 16), 'g': int(color[3:5], 16), 'b': int(color[5:7], 16)}`;

// LED blocks: the LED strip driver is not ported to ROS 2 yet; the API raises
// NotSupported and technic_blocks refuses to start such programs.
// TODO: weird code with colour_rgb block
Blockly.Python.set_effect = function(block) {
	initNode('set_effect');

	var effect = block.getFieldValue('EFFECT').toLowerCase();

	if (effect == 'rainbow' || effect == 'rainbow_fill') {
		return `set_effect(effect='${effect}')\n`;
	} else {
		let colorCode = Blockly.Python.valueToCode(block, 'COLOR', Blockly.Python.ORDER_NONE);

		if (/^'(.*)'$/.test(colorCode)) { // is simple string
			let color = parseColor(colorCode);
			return `set_effect(effect='${effect}', r=${color.r}, g=${color.g}, b=${color.b})\n`;
		} else {
			let parseColor = Blockly.Python.provideFunction_('parse_color', [PARSE_COLOR]);
			return `set_effect(effect='${effect}', **${parseColor}(${colorCode}))\n`;
		}
	}
}

Blockly.Python.set_led = function(block) {
	initNode('set_leds', 'LEDState');

	var index = Blockly.Python.valueToCode(block, 'INDEX', Blockly.Python.ORDER_NONE);
	var colorCode = Blockly.Python.valueToCode(block, 'COLOR', Blockly.Python.ORDER_NONE);

	if (/^'(.*)'$/.test(colorCode)) { // is simple string
		let color = parseColor(colorCode);
		return `set_leds([LEDState(index=int(${index}), r=${color.r}, g=${color.g}, b=${color.b})])\n`; // TODO: check for simple int
	} else {
		let parseColor = Blockly.Python.provideFunction_('parse_color', [PARSE_COLOR]);
		return `set_leds([LEDState(index=${index}, **${parseColor}(${colorCode}))])\n`;
	}
}

Blockly.Python.led_count = function(block) {
	initNode('get_led_count');
	return [`get_led_count()`, Blockly.Python.ORDER_FUNCTION_CALL]
}

function technic_gpio() {
	Blockly.Python.definitions_['import_technic_gpio'] =
		'from technic_blocks.gpio import gpio_read, gpio_write, set_servo, set_duty_cycle';
}

Blockly.Python.gpio_read = function(block) {
	technic_gpio();
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	return [`gpio_read(${pin})`, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.gpio_write = function(block) {
	technic_gpio();
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	var level = Blockly.Python.valueToCode(block, 'LEVEL', Blockly.Python.ORDER_NONE);
	return `gpio_write(${pin}, ${level})\n`;
}

Blockly.Python.set_servo = function(block) {
	technic_gpio();
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	var pwm = Blockly.Python.valueToCode(block, 'PWM', Blockly.Python.ORDER_NONE);
	return `set_servo(${pin}, ${pwm})\n`;
}

Blockly.Python.set_duty_cycle = function(block) {
	technic_gpio();
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	var dutyCycle = Blockly.Python.valueToCode(block, 'DUTY_CYCLE', Blockly.Python.ORDER_NONE);
	return `set_duty_cycle(${pin}, ${dutyCycle})\n`;
}
