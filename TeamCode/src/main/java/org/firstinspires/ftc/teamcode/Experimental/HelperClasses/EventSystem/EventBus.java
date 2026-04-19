package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.EventSystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class EventBus {
    private final Map<Class<? extends Event>, List<EventListener<? extends Event>>> listeners = new HashMap<>();

    public <T extends Event> void subscribe(Class<T> type, EventListener<T> listener) {
        listeners.computeIfAbsent(type, k -> new ArrayList<>())
                .add(listener);
    }

    public <T extends Event> void unsubscribe(Class<T> type, EventListener<T> listener) {
        List<EventListener<? extends Event>> list = listeners.get(type);
        if (list != null) {
            list.remove(listener);
        }
    }

    @SuppressWarnings("unchecked")
    public <T extends Event> void emit(T event) {
        List<EventListener<? extends Event>> list = listeners.get(event.getClass());

        if (list == null) return;

        for (EventListener<? extends Event> listener : list) {
            ((EventListener<T>) listener).onEvent(event);
            if (event.isCancelled()) break;
        }
    }
}
