<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import SettingsCard from '$lib/components/SettingsCard.svelte';
  import Light from '~icons/tabler/bulb';
  import Info from '~icons/tabler/info-circle';
  import { socket } from '$lib/stores/socket';
  import type { LightState, DemoTelemetry, SequenceStatus } from '$lib/types/models';
  import SDK from '~icons/tabler/ruler-2';
  import CPU from '~icons/tabler/scale';

  // Local state for LED and timers
  let lightState: LightState = { led_on: false, retect_seconds: 30, feed_seconds: 30, target_distance_cm: 30, return_distance_cm: 30 };
  // Sequence running flag to disable UI while active
  let running = false;
  let status: SequenceStatus = { phase: 'standby', running: false, distance_mm: 0 };
  // Auto sequence interval (minutes). 0 = off
  let autoIntervalMin: number = 0;

  // Live telemetry state
  let weight_g: number = 0;
  let distance_mm: number = 0; // 0 => out of range
  let endstop: boolean = false;
  let di_mask: number | undefined = undefined;
  let next_start_in_s: number | undefined = undefined;

  // Overlay/timeout logic
  let overlay = true; // start greyed out until data arrives
  let staleTimer: ReturnType<typeof setTimeout> | null = null;
  const DATA_STALE_MS = 5000; // blur if no data for 5s (less flicker)
  function armStaleTimer() {
    if (staleTimer) clearTimeout(staleTimer);
    staleTimer = setTimeout(() => {
      overlay = true;
    }, DATA_STALE_MS);
  }

  onMount(() => {
    // Listen for server updates
    socket.on<LightState>('led', handleSystemData);
    socket.on<DemoTelemetry>('demo', handleDemo);
    socket.on<SequenceStatus>('sequence_status', (s) => {
      status = s;
      running = s.running;
    });
    // schedule status
    socket.on('sequence_schedule_status', (s: { interval_min: number; enabled: boolean }) => {
      if (typeof s?.interval_min === 'number') autoIntervalMin = s.interval_min;
    });
    // socket lifecycle events
    socket.on('open', () => {
      // give connection a grace window before greying out again
      armStaleTimer();
    });
    socket.on('unresponsive', handleStale);
    socket.on('close', handleStale);
    socket.on('error', handleStale);
  // ask device for current schedule status
  socket.sendEvent('sequence_schedule_get', {});
  });

  const handleSystemData = (data: LightState) => {
    lightState = { ...lightState, ...data };
    if (typeof data.auto_interval_min === 'number') {
      autoIntervalMin = data.auto_interval_min;
    }
  };

  const handleDemo = (data: DemoTelemetry) => {
    weight_g = data.weight_g ?? weight_g;
    distance_mm = data.distance_mm ?? distance_mm;
  if (typeof data.endstop === 'boolean') endstop = data.endstop;
  if (typeof data.di_mask === 'number') di_mask = data.di_mask;
  if (typeof data.next_start_in_s === 'number') next_start_in_s = data.next_start_in_s;
    // data arrived -> unblur and arm stale timer
    overlay = false;
    armStaleTimer();
  };

  const handleStale = () => {
    // Don't flip overlay immediately; give a grace window and
    // only grey out if no data arrives within DATA_STALE_MS
    armStaleTimer();
  };

  onDestroy(() => {
    socket.off('led', handleSystemData);
    socket.off('demo', handleDemo);
  socket.off('sequence_status');
    socket.off('open');
    socket.off('unresponsive', handleStale);
    socket.off('close', handleStale);
    socket.off('error', handleStale);
    if (staleTimer) clearTimeout(staleTimer);
  });

  async function startSequence() {
    if (running) return;
    running = true;
    // Fire the sequence request with current parameters
    const payload = {
  target_distance_cm: lightState.target_distance_cm ?? 30,
  feed_seconds: lightState.feed_seconds,
  return_distance_cm: lightState.return_distance_cm ?? 30
    };
    console.log('[UI] start sequence', payload);
    socket.sendEvent('sequence', payload);
    // Running state will be updated by sequence_status events
  }

  function abortSequence() {
    socket.sendEvent('sequence_abort', {});
  }

  // Jog/Home/Feed manual controls
  function jogUpStart() {
    socket.sendEvent('jog', { dir: 'up' });
  }
  function jogDownStart() {
    socket.sendEvent('jog', { dir: 'down' });
  }
  function jogStop() {
    // Stop any manual motion/feed tasks
    socket.sendEvent('jog_stop', {});
    socket.sendEvent('feed_stop', {});
  }
  function feedStart() {
    // Start continuous feed until stopped
    socket.sendEvent('feed', {});
  }
  function returnHome() {
    // Return to home (top) position: we consider UI's return_distance_cm as home target
    const cm = lightState.return_distance_cm ?? 30;
    socket.sendEvent('home', { target_distance_cm: cm });
  }

  // Function to adjust feed_seconds via slider
  function adjFeedSeconds(seconds: number) {
    lightState = { ...lightState, feed_seconds: seconds };
    console.log('[UI] sendEvent Feed Seconds', lightState);
    socket.sendEvent('led', lightState);
  }

  // Function to adjust retect_seconds via slider
  function adjRetectSeconds(seconds: number) {
    lightState = { ...lightState, retect_seconds: seconds };
    // Persist retect_seconds to backend
    socket.sendEvent('led', lightState);
  }

  function formatDistance(mm: number): string {
    if (!mm) return 'Out of range';
    return `${(mm / 10).toFixed(1)} cm`;
  }

  function setAutoInterval(min: number) {
    if (min < 0) min = 0;
    if (min > 60) min = 60;
    autoIntervalMin = min;
    // Persist along with other demo settings
    lightState = { ...lightState, auto_interval_min: autoIntervalMin };
    socket.sendEvent('led', lightState);
    // Also apply immediately to scheduler
    socket.sendEvent('sequence_schedule', { interval_min: autoIntervalMin });
  }
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <Light class="lex-shrink-0 mr-2 h-6 w-6 self-end" />
  {/snippet}
  {#snippet title()}
    <span>Instillinger</span>
  {/snippet}

  <div class="relative">
    <div class="w-full transition-all duration-200" class:blur-md={overlay} class:grayscale={overlay} class:opacity-60={overlay}>
      <h1 class="text-xl font-semibold">Eksempel applikasjon</h1>
      <div class="alert alert-info my-2 shadow-lg">
        <Info class="h-6 w-6 shrink-0 stroke-current" />
        <span>
          Test foringsautomat, klikk for å starte sekvens
        </span>
      </div>

      <!-- Live values -->
      <div class="grid grid-cols-2 gap-3 mb-4">
        <div class="rounded-box bg-base-100 flex items-center space-x-3 px-4 py-2">
          <div class="mask mask-hexagon bg-primary h-auto w-10 flex-none">
            <CPU class="text-primary-content h-auto w-full scale-75" />
          </div>
          <div>
            <div class="font-bold">Innhold</div>
            <div class="text-sm opacity-75">
              {weight_g} g
            </div>
          </div>
        </div>

        <div class="rounded-box bg-base-100 flex items-center space-x-3 px-4 py-2">
          <div class="mask mask-hexagon bg-primary h-auto w-10 flex-none">
            <SDK class="text-primary-content h-auto w-full scale-75" />
          </div>
          <div>
            <div class="font-bold">Posisjon</div>
            <div class="text-sm opacity-75">
              {formatDistance(distance_mm)}
            </div>
          </div>
        </div>
      </div>


      <div class="w-full mt-2 max-w-sm flex gap-2">
        <button
          class="btn btn-primary flex-1"
          disabled={running}
          onclick={startSequence}
        >
          {running ? 'Kjører sekvens…' : 'Start sekvens'}
        </button>
        <button class="btn btn-error flex-none" disabled={!running} onclick={abortSequence}>Nødstopp</button>
      </div>

      <!-- Manual controls: Jog/Feed/Home -->
      <div class="mt-3 grid grid-cols-2 md:grid-cols-4 gap-2 max-w-xl">
        <button class="btn btn-secondary" onclick={jogUpStart} disabled={running}>Jog opp</button>
        <button class="btn btn-secondary" onclick={jogDownStart} disabled={running}>Jog ned</button>
        <button class="btn btn-success" onclick={feedStart} disabled={running}>Jog mater</button>
        <button class="btn btn-warning" onclick={jogStop}>Jog Stopp</button>
        <button class="btn btn-accent col-span-2 md:col-span-1" onclick={returnHome} disabled={running}>Returner hjem</button>
      </div>

  <!-- Target Down Distance, Feed Seconds and Return Distance Sliders in Horizontal Layout -->
  <div class="mt-4 grid grid-cols-1 md:grid-cols-3 gap-6">
        <!-- Target (Lower) Distance Slider -->
        <div class="flex-1">
          <label for="target_distance_cm" class="block text-sm font-medium text-gray-700">Mate posisjon (cm)</label>
          <input 
            type="range" 
            id="target_distance_cm" 
            min="10" 
            max="200" 
            step="1" 
            bind:value={lightState.target_distance_cm}
            oninput={(e) => {
              const target = e.target as HTMLInputElement | null;
              if (target) {
                const cm = Number(target.value);
                lightState = { ...lightState, target_distance_cm: cm };
                // Persist to backend
                socket.sendEvent('led', lightState);
              }
            }}
            class="mt-2 w-full bg-gray-200 rounded-lg h-2 focus:outline-none focus:ring-2 focus:ring-indigo-500"
          />
          <div class="mt-2 text-center text-sm text-gray-600">
            {lightState.target_distance_cm ?? 30} cm
          </div>
        </div>
        <!-- Feed Seconds Slider -->
        <div class="flex-1">
          <label for="feed_seconds" class="block text-sm font-medium text-gray-700">Matetid (sekunder)</label>
            <input 
              type="range" 
              id="feed_seconds" 
              min="0" 
              max="120" 
              step="1" 
              bind:value={lightState.feed_seconds}
              oninput={(e) => {
                const target = e.target as HTMLInputElement | null;
                if (target) adjFeedSeconds(Number(target.value));
              }}
              class="mt-2 w-full bg-gray-200 rounded-lg h-2 focus:outline-none focus:ring-2 focus:ring-indigo-500"
            />
            <div class="mt-2 text-center text-sm text-gray-600">
              {lightState.feed_seconds} sekunder
            </div>
        </div>

        <!-- Return Distance Slider -->
        <div class="flex-1">
          <label for="return_distance_cm" class="block text-sm font-medium text-gray-700">Hjem posisjon (cm)</label>
          <input
            type="range"
            id="return_distance_cm"
            min="10"
            max="200"
            step="1"
            bind:value={lightState.return_distance_cm}
            oninput={(e) => {
              const target = e.target as HTMLInputElement | null;
              if (target) {
                const cm = Number(target.value);
                lightState = { ...lightState, return_distance_cm: cm };
                socket.sendEvent('led', lightState);
              }
            }}
            class="mt-2 w-full bg-gray-200 rounded-lg h-2 focus:outline-none focus:ring-2 focus:ring-indigo-500"
          />
          <div class="mt-2 text-center text-sm text-gray-600">
            {lightState.return_distance_cm ?? 30} cm
          </div>
        </div>
      </div>

      <!-- Auto sequence interval slider -->
      <div class="mt-4 max-w-sm">
        <label for="auto_interval_min" class="block text-sm font-medium text-gray-700">Auto-sekvens intervall (minutter)</label>
        <input
          type="range"
          id="auto_interval_min"
          min="0"
          max="60"
          step="1"
          bind:value={autoIntervalMin}
          oninput={(e) => {
            const t = e.target as HTMLInputElement | null;
            if (t) setAutoInterval(Number(t.value));
          }}
          class="mt-2 w-full bg-gray-200 rounded-lg h-2 focus:outline-none focus:ring-2 focus:ring-indigo-500"
        />
        <div class="mt-2 text-center text-sm text-gray-600">
          {autoIntervalMin === 0 ? 'Av' : `${autoIntervalMin} min`}
        </div>
      </div>

      <!-- Sequence status -->
      <div class="mt-4">
  <div class="badge badge-outline mr-2">Status: {status.phase}</div>
  {#if autoIntervalMin > 0 && typeof next_start_in_s === 'number'}
          <div class="badge badge-info mr-2">Auto om {next_start_in_s}s</div>
        {/if}
  {#if endstop}
          <div class="badge badge-error">Endestopp aktiv</div>
        {/if}
        {#if di_mask !== undefined}
          <div class="badge badge-ghost ml-2">DI: {di_mask}</div>
        {/if}
      </div>
    </div>

    {#if overlay}
      <div class="absolute inset-0 bg-base-300/40 grid place-items-center transition-opacity duration-200">
        <div class="rounded-box bg-base-100/90 px-4 py-3 shadow text-center">
          <div class="font-semibold">Venter på data fra enheten…</div>
          <div class="text-sm opacity-70">Sjekker tilkobling</div>
        </div>
      </div>
    {/if}
  </div>
</SettingsCard>
