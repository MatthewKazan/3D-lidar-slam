create table point_cloud
(
    x       double precision not null,
    y       double precision not null,
    z       double precision not null,
    scan_id integer
);

alter table point_cloud
    owner to newuser;

create index idx_scan_id
    on point_cloud (scan_id);