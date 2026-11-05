const url = 'ws://' + location.hostname + ':9090';
const ros = new ROSLIB.Ros({ url: url });
const params = Object.fromEntries(new URLSearchParams(window.location.search).entries());

ros.on('connection', function () {
	document.body.classList.add('connected');
	document.body.classList.remove('closed');
	init();
});

ros.on('close', function () {
	document.body.classList.remove('connected');
	document.body.classList.add('closed');
	setTimeout(function() {
		// reconnect
		ros.connect(url);
	}, 2000);
});

const title = document.querySelector('h1');
const topicsList = document.querySelector('#topics');
const topicMessage = document.querySelector('#topic-message');

/** Видеокодеки web_video_server, кодирующие из сырого топика. */
const WVS_VIDEO_CODECS = ['h264', 'vp8', 'vp9'];

/** ROS 2: отдельных топиков image_web нет, web_video_server берёт сам image_raw
 * (ros_compressed — его транспорт image_raw/compressed). */
function webVideoCompressedTopic(imageTopic) {
	return imageTopic;
}

function webVideoRawTopic(imageTopic) {
	return imageTopic;
}

/** Ссылка на web_video_server для топика. По умолчанию H264 (низкая задержка, MP4 в браузере). */
function webVideoStreamUrl(imageTopic) {
	const type = params.wvs_type || 'h264';
	const isVideoCodec = WVS_VIDEO_CODECS.includes(type);
	const p = new URLSearchParams({
		topic: isVideoCodec ? webVideoRawTopic(imageTopic) : webVideoCompressedTopic(imageTopic),
		type: type,
		width: params.wvs_w || '320',
		height: params.wvs_h || '240'
	});
	if (isVideoCodec) {
		p.set('bitrate', params.wvs_bitrate || '300000');
	} else if (params.wvs_quality) {
		p.set('quality', params.wvs_quality);
	}
	// web_video_server не раскодирует %2F в имени топика, поэтому слэши оставляем как есть
	const query = p.toString().replace(/%2F/gi, '/');
	return `${location.protocol}//${location.hostname}:8080/stream_viewer?${query}`;
}

function viewTopicsList() {
	title.innerHTML = 'Topics:';

	ros.getTopics(function(topics) {
		const items = topics.topics.map(function(topic, i) {
			return { topic: topic, type: topics.types[i] };
		}).sort(function(a, b) {
			return a.topic.localeCompare(b.topic);
		});

		topicsList.innerHTML = items.map(function(item) {
			const topic = item.topic;
			const type = item.type;
			if (type == 'sensor_msgs/Image') {
				const url = webVideoStreamUrl(topic);
				return `<li><a href="${url}" class=topic title=${type}>${topic}</a></li>`;
			} else {
				return `<li><a href="?topic=${topic}" class=topic title=${type}>${topic}</a></li>`;
			}
		}).join('');
	});
}

let rosdistro = 'jazzy';

function viewTopic(topic) {
	let counter = 0;
	let index = '<a href=topics.html>Topics</a>';
	title.innerHTML = `${index}: ${topic}`;
	topicMessage.style.display = 'block';

	ros.getTopicType(topic, function(typeStr) {
		// ROS 2 type names: package/msg/Type
		const parts = typeStr.split('/');
		const pack = parts[0];
		const type = parts[parts.length - 1];
		let href = `https://docs.ros.org/en/${rosdistro}/p/${pack}/interfaces/msg/${type}.html`;
		title.innerHTML = `${index}: ${topic} <a id="topic-type" href=${href} target="_blank">${typeStr}</a>`;
	});

	new ROSLIB.Topic({ ros: ros, name: topic }).subscribe(function(msg) {
		counter++;
		document.title = topic;
		if (mouseDown) return;

		if (msg.header && msg.header.stamp) {
			if (params.date || params.offset) {
				let date = new Date(msg.header.stamp.sec * 1e3 + msg.header.stamp.nanosec * 1e-6);
				if (params.date) msg.header.date = date.toISOString();
				if (params.offset) msg.header.offset = (new Date() - date) * 1e-3;
			}
		}

		let width = Number(params.width) || 100;
		let indent = Number(params.indent) || 2;
		let txt = YAML.stringify(msg, { lineWidth: width, indent: indent });
		let html = `<div class=counter>${counter} received</div>${txt}`; // JSON.stringify(msg, null, 4);
		topicMessage.innerHTML = html;
	});
}

let mouseDown;

topicMessage.addEventListener('mousedown', function() { mouseDown = true; });
topicMessage.addEventListener('mouseup', function() { mouseDown = false; });

function init() {
	if (!params.topic) {
		viewTopicsList();
	} else {
		new ROSLIB.Service({ ros: ros, name: '/rosapi/get_ros_version', serviceType: 'rosapi_msgs/srv/GetROSVersion' })
			.callService(new ROSLIB.ServiceRequest({}), function(res) {
				rosdistro = (res.distro || rosdistro).trim();
				viewTopic(params.topic);
			}, function() {
				viewTopic(params.topic);
			});
	}
}
