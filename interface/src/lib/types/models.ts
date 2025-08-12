export type WifiStatus = {
	status: number;
	local_ip: string;
	mac_address: string;
	rssi: number;
	ssid: string;
	bssid: string;
	channel: number;
	subnet_mask: string;
	gateway_ip: string;
	dns_ip_1: string;
	dns_ip_2?: string;
};

export type WifiSettings = {
	hostname: string;
	connection_mode: number;
	wifi_networks: KnownNetworkItem[];
};

export type KnownNetworkItem = {
	ssid: string;
	password: string;
	static_ip_config: boolean;
	local_ip?: string;
	subnet_mask?: string;
	gateway_ip?: string;
	dns_ip_1?: string;
	dns_ip_2?: string;
};

export type NetworkItem = {
	rssi: number;
	ssid: string;
	bssid: string;
	channel: number;
	encryption_type: number;
};

export type ApStatus = {
	status: number;
	ip_address: string;
	mac_address: string;
	station_num: number;
};

export type ApSettings = {
	provision_mode: number;
	ssid: string;
	password: string;
	channel: number;
	ssid_hidden: boolean;
	max_clients: number;
	local_ip: string;
	gateway_ip: string;
	subnet_mask: string;
};

export type LightState = {
	led_on: boolean;
	retect_seconds: number;
	feed_seconds: number;
	target_distance_cm?: number;
	return_distance_cm?: number;
	auto_interval_min?: number; // 0 = off, 1-60 = minutes between automatic runs

};

// Live telemetry payload from firmware (event: "demo")
export type DemoTelemetry = {
	weight_g: number;      // current weight in grams
	distance_mm: number;   // current distance in millimeters (0 = out of range)
	endstop?: boolean;     // optional: endstop state
	di_mask?: number;      // optional: bitmask of DI inputs (debug)
	next_start_in_s?: number; // optional: seconds until next auto-start (when enabled)
};

export interface SequenceParams {
	target_distance_cm: number; // stop when distance sensor reaches this
	feed_seconds: number;       // run feeder (relay 4)
	return_distance_cm: number; // absolute distance target to return to after feed
}

export type SequencePhase =
	| 'standby'
	| 'approach'
	| 'feed'
	| 'retract'
	| 'manual_feed'
	| 'jog_up'
	| 'jog_down'
	| 'home'
	| 'done'
	| 'aborted'
	| 'endstop';

export type SequenceStatus = {
	phase: SequencePhase;
	running: boolean;
	distance_mm: number;
	start_distance_mm?: number;
	target_down_mm?: number;
	return_target_mm?: number;
	endstop?: boolean;
	di_mask?: number;
	reason?: string;
};

export type BrokerSettings = {
	mqtt_path: string;
	name: string;
	unique_id: string;
	status_topic: string;
};

export type NTPStatus = {
	status: number;
	utc_time: string;
	local_time: string;
	server: string;
	uptime: number;
};

export type NTPSettings = {
	enabled: boolean;
	server: string;
	tz_label: string;
	tz_format: string;
};

export type Analytics = {
	max_alloc_heap: number;
	psram_size: number;
	free_psram: number;
	used_psram: number;
	free_heap: number;
	used_heap: number;
	total_heap: number;
	min_free_heap: number;
	core_temp: number;
	fs_total: number;
	fs_used: number;
	uptime: number;
};

export type RSSI = {
	rssi: number;
	ssid: string;
};

export type Battery = {
	soc: number;
	charging: boolean;
};

export type DownloadOTA = {
	status: string;
	progress: number;
	error: string;
};

export type StaticSystemInformation = {
	esp_platform: string;
	firmware_version: string;
	cpu_freq_mhz: number;
	cpu_type: string;
	cpu_rev: number;
	cpu_cores: number;
	sketch_size: number;
	free_sketch_space: number;
	sdk_version: string;
	arduino_version: string;
	flash_chip_size: number;
	flash_chip_speed: number;
	cpu_reset_reason: string;
};

export type SystemInformation = Analytics & StaticSystemInformation;

export type MQTTStatus = {
	enabled: boolean;
	connected: boolean;
	client_id: string;
	last_error: string;
};

export type MQTTSettings = {
	enabled: boolean;
	uri: string;
	username: string;
	password: string;
	client_id: string;
	keep_alive: number;
	clean_session: boolean;
};
